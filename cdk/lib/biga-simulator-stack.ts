var _ = require('lodash');
import * as fs from 'fs';
import path = require('path');

import * as cdk from 'aws-cdk-lib';
import { Construct } from 'constructs';
import * as ec2 from 'aws-cdk-lib/aws-ec2';
import * as iam from 'aws-cdk-lib/aws-iam';
import * as secret from 'aws-cdk-lib/aws-secretsmanager';
import * as assets from 'aws-cdk-lib/aws-s3-assets';

const SIMULATOR_CONFIG_DESTINATION = '/opt/simulator-config';

function pascalCase(input: string): string {
  return _.upperFirst(_.camelCase(input));
}
function readAndreplaceWithDict(filePath: string, vars: any): string {
  const content = fs.readFileSync(filePath, 'utf-8');
  return content.replace(/\${(\w+)}/g, (_, key) => vars[key] || '');
}

/**
 * Select options for the {@link PipelineResourcesStack}.
 */
export interface BigaSimulatorProps extends cdk.StackProps {
  readonly resourcePrefix: string;

  readonly allowedCIDR: string;
  allowedAvailabilityZone?: string;

  readonly bigaMachineImage?: ec2.IMachineImage;
  readonly bigaInstanceType?: string;

  readonly carlaMachineImage?: ec2.IMachineImage;
  readonly carlaInstanceType?: string;
  readonly carlaInstanceVolumeSize?: number;
  readonly carlaOSUserName?: string;
  readonly carlaVersion?: string;

  readonly useMultiCast: boolean;
}

const { execSync } = require('child_process');
const cmd = `curl -s http://checkip.amazonaws.com || printf "0.0.0.0"`;
const currentIP = `${execSync(cmd).toString().trim()}/32`;

const defaultProps: BigaSimulatorProps = {
  resourcePrefix: 'biga-simulator',

  allowedCIDR: currentIP,

  bigaInstanceType: "t4g.micro",
  bigaMachineImage: undefined,

  carlaInstanceType: "g5.16xlarge",
  carlaInstanceVolumeSize: 40,
  carlaMachineImage: undefined,
  carlaOSUserName: 'biga',
  carlaVersion: '0.9.13',

  useMultiCast: true,
};

export class BigaSimulatorStack extends cdk.Stack {
  constructor(scope: Construct, id: string, _props: BigaSimulatorProps) {
    super(scope, id, { ...defaultProps, ..._props });

    // get the first az from the region in case ze don't provide one
    defaultProps.allowedAvailabilityZone = cdk.Stack.of(this).availabilityZones[0];
    const props = { ...defaultProps, ..._props };

    const carlaValidInstanceTypes = ['g4dn.xlarge',
      'g4dn.2xlarge',
      'g4dn.4xlarge',
      'g4dn.8xlarge',
      'g4dn.12xlarge',
      'g4dn.16xlarge',
      'g5.xlarge',
      'g5.2xlarge',
      'g5.4xlarge',
      'g5.8xlarge',
      'g5.12xlarge',
      'g5.16xlarge'
    ];
    if (!carlaValidInstanceTypes.includes(props.carlaInstanceType!)) {
      throw new Error(`Carla instance type ${props.carlaInstanceType} is not allowed. Allowed instance types are : ${carlaValidInstanceTypes}`);
    }

    try {
      const bigaInstanceType = new ec2.InstanceType(props.bigaInstanceType!);
      if (bigaInstanceType.architecture != ec2.InstanceArchitecture.ARM_64) {
        throw new Error(`Biga instance only support ${ec2.InstanceArchitecture.ARM_64} architecture, but current instance type is : ${props.carlaInstanceType}.`);
      }
    } catch (e) {
      throw new Error(`Biga instance type ${props.carlaInstanceType} doesn't exists.`);
    }

    var vpc: ec2.Vpc;
    var vpcSecurityGroup: ec2.SecurityGroup;

    var bigaInstance: ec2.CfnInstance;
    var bigaNetworkInterface: ec2.CfnNetworkInterface;

    var carlaInstance: ec2.CfnInstance;
    var carlaSecret: secret.Secret;
    var carlaInstanceIAMRole: cdk.aws_iam.Role;
    var carlaInstanceIAMInstanceProfile: cdk.aws_iam.InstanceProfile;
    var carlaNetworkInterface: ec2.CfnNetworkInterface;

    var vpcTransitGateway: ec2.CfnTransitGateway;
    var vpcTransitGatewayAttachment: ec2.CfnTransitGatewayAttachment;
    var vpcTransitGatewayDomain: ec2.CfnTransitGatewayMulticastDomain;
    var vpcTransitGatewayGroupMemberCarla: ec2.CfnTransitGatewayMulticastGroupMember;
    var vpcTransitGatewayGroupMemberBiga: ec2.CfnTransitGatewayMulticastGroupMember;

    const envVariables: { [key: string]: string; } = {
      // stack details
      STACK_REGION: cdk.Stack.of(this).region,
      STACK_ID: cdk.Stack.of(this).stackId,
      STACK_NAME: cdk.Stack.of(this).stackName,
      STACK_ACCOUNT: cdk.Stack.of(this).account,
      // props
      CARLA_VERSION: props.carlaVersion!,
      CARLA_OS_USER_NAME: props.carlaOSUserName!,
    };

    // define the vpc, security group & network interfaces
    {
      vpc = new ec2.Vpc(this, `vpc`, {
        vpcName: `${props.resourcePrefix}-vpc`,
        enableDnsHostnames: true,
        enableDnsSupport: true,
        subnetConfiguration: [
          {
            name: `${props.resourcePrefix}-public-subnet`,
            subnetType: ec2.SubnetType.PUBLIC,
            cidrMask: 24,
          },
        ],
        maxAzs: 2
      });

      vpcSecurityGroup = new ec2.SecurityGroup(this, `vpc-sg`, {
        vpc: vpc,
        description: "Allow Traffic From/To Carla & Biga instances",
        allowAllOutbound: true,
        securityGroupName: `${props.resourcePrefix}-vpc-sg`,
      });
      cdk.Tags.of(vpcSecurityGroup).add('Name', `${props.resourcePrefix}-vpc-sg`);

      vpcSecurityGroup.addIngressRule(ec2.Peer.ipv4(vpc.vpcCidrBlock), ec2.Port.tcp(22), "SSH from anywhere within the VPC");
      vpcSecurityGroup.addIngressRule(ec2.Peer.ipv4(props.allowedCIDR), ec2.Port.tcp(8443), "DCV access from the outside - tcp");
      vpcSecurityGroup.addIngressRule(ec2.Peer.ipv4(props.allowedCIDR), ec2.Port.udp(8443), "DCV access from the outside - udp");
    }

    // define the carla simulator ec2 instance
    {
      carlaSecret = new secret.Secret(this, `carla-secret`, {
        secretName: `${props.resourcePrefix}-carla-secret`,
        description: "Simple secret created by AWS CDK for the Carla instance.",
        generateSecretString: {
          secretStringTemplate: `{"username": "${props.carlaOSUserName}"}`,
          generateStringKey: "password",
          excludeCharacters: "`'\"@/\\;,$%<>^",
          passwordLength: 32,
        }
      });

      carlaInstanceIAMRole = new iam.Role(this, `carla-instance-iam-role`, {
        roleName: `${props.resourcePrefix}-carla-instance-iam-role`,
        assumedBy: new iam.ServicePrincipal('ec2.amazonaws.com'),
        description: 'IAM Role for the Carla EC2 instance.',
        managedPolicies: [
          iam.ManagedPolicy.fromAwsManagedPolicyName('AmazonSSMManagedInstanceCore'),
        ],
        inlinePolicies: {
          'dcvLicensing': new iam.PolicyDocument({
            statements: [
              new iam.PolicyStatement({
                effect: iam.Effect.ALLOW,
                actions: [
                  's3:GetObject',
                ],
                resources: [
                  `arn:${cdk.Aws.PARTITION}:s3:::dcv-license.${cdk.Aws.REGION}/*`
                ],
              })
            ]
          }),
          'secret': new iam.PolicyDocument({
            statements: [
              new iam.PolicyStatement({
                effect: iam.Effect.ALLOW,
                actions: [
                  'secretsmanager:GetResourcePolicy',
                  'secretsmanager:GetSecretValue',
                  'secretsmanager:DescribeSecret',
                  'secretsmanager:ListSecret*'
                ],
                resources: [
                  carlaSecret.secretArn
                ],
              })
            ]
          })
        }
      });
      carlaInstanceIAMInstanceProfile = new iam.InstanceProfile(this, `carla-instance-iam-instance-profile`, {
        role: carlaInstanceIAMRole,
        instanceProfileName: `${props.resourcePrefix}-carla-instance-iam-instance-profile`
      });
      carlaNetworkInterface = new ec2.CfnNetworkInterface(this, `carla-instance-eip`, {
        subnetId: vpc.publicSubnets[0].subnetId,
        description: "Simulator Carla Network Interface",
        groupSet: [vpcSecurityGroup.securityGroupId],
        tags: [{
          key: 'Name',
          value: `${props.resourcePrefix}-carla-instance-eip`,
        }],
      });
      carlaInstance = new ec2.CfnInstance(this, `carla-instance`, {
        tags: [{ key: 'Name', value: `${props.resourcePrefix}-instance-carla` }],

        instanceType: props.carlaInstanceType!,
        imageId: props.carlaMachineImage!.getImage(this).imageId,
        monitoring: true,
        blockDeviceMappings: [
          {
            deviceName: '/dev/sda1',
            ebs: {
              volumeType: ec2.EbsDeviceVolumeType.GP3,
              volumeSize: props.carlaInstanceVolumeSize,
              deleteOnTermination: true,
            },
          }
        ],
        iamInstanceProfile: carlaInstanceIAMInstanceProfile.instanceProfileName,
        networkInterfaces: [{
          deviceIndex: "0",
          networkInterfaceId: carlaNetworkInterface.ref,
        }],
      });
      // add secrets dependency
      carlaInstance.node.addDependency(carlaSecret);

      const simulatorConfigS3Asset = new assets.Asset(this, `${props.resourcePrefix}-carla-instance-cloud-init-asset`, {
        path: '../simulator-config',
        displayName: 'simulator-config',
        deployTime: true,
        readers: [carlaInstanceIAMRole]
      });

      // adding entries for Fn sub
      envVariables['CARLA_SECRET'] = carlaSecret.secretName;
      envVariables['SIMULATOR_CONFIG_DESTINATION'] = SIMULATOR_CONFIG_DESTINATION;
      envVariables['S3_BUCKET'] = simulatorConfigS3Asset.s3BucketName;
      envVariables['S3_OBJECT'] = simulatorConfigS3Asset.s3ObjectKey;
      envVariables['STACK_RESOURCE_iD'] = carlaInstance.logicalId;/*node.id.replace('-', '').replace('_', '') */

      // adding CloudFormation Init
      {
        // adding user data
        const userData = ec2.UserData.forLinux({
          shebang: '#!/bin/bash'
        });
        userData.addCommands(...[
          'export STACK_NAME="${STACK_NAME}"',
          'export STACK_REGION="${STACK_REGION}"',
          'export STACK_RESOURCE_iD="${STACK_RESOURCE_iD}"',
          '',
          'export DEBIAN_FRONTEND=noninteractive',
          '',
          'while apt-get upgrade -y | grep -q "Could not get lock" ; do echo "Waiting for other apt-get instances to exit"; sleep 1; done',
          'while sudo fuser /var/{lib/{dpkg,apt/lists},cache/apt/archives}/lock >/dev/null 2>&1; do echo "Waiting for other apt-get instances to exit"; sleep 1; done',
          'add-apt-repository ppa:deadsnakes/ppa -y',
          'apt-get -qq -y update',
          `apt-get -qq -y install \
              python3 \
              python-is-python3 \
              python3-pip
          `,
          '',
          'python -m pip install https://s3.amazonaws.com/cloudformation-examples/aws-cfn-bootstrap-py3-latest.tar.gz -q -q -q',
          '',
          '# Start cfn-init',
          `cfn-init -v --stack $STACK_NAME --resource $STACK_RESOURCE_iD --region $STACK_REGION -c default`,
          'STATUS=$?',
          '',
          'echo "cfn-init STATUS = $STATUS"',
          '',
          '# Start up the cfn-hup daemon to listen for changes to the EC2 instance metadata',
          '/opt/aws/bin/cfn-hup',
          '',
          '# cfn-init completed so signal success or not',
          `cfn-signal -e $STATUS --stack $STACK_NAME --resource $STACK_RESOURCE_iD --region $STACK_REGION`,
          '',
          '# reboot for all the changes to take effect',
          'reboot',
        ]);
        carlaInstance.userData = cdk.Fn.base64(
          cdk.Fn.sub(userData.render(), envVariables)
        );
        const cloudInitFileOptions = {
          mode: "000755",
          owner: "root",
          group: "root",
        };
        const commandsPath = path.join(__dirname, '../../simulator-config/assets/commands');
        const commandsFiles = fs.readdirSync(commandsPath).sort(
        ).map(file => ({
          ...path.parse(file),
          'file': file,
          'fullPath': path.join(commandsPath, file),
        })
        ).filter(item => {
          return fs.statSync(item['fullPath']).isFile();
        });

        const cfnInit = ec2.CloudFormationInit.fromConfigSets({
          configSets: {
            default: ['commands']
          },
          configs: {
            commands: new ec2.InitConfig(
              [
                ec2.InitCommand.shellCommand(
                  [
                    `
                    apt update
                    apt install -y -q unzip

                    curl "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o "awscliv2.zip"
                    unzip -qq awscliv2.zip
                    ./aws/install
                    rm awscliv2.zip

                    echo "step: install awscliv2 ok" >> /tmp/my-cloud-int.log 

                    mkdir -p $SIMULATOR_CONFIG_DESTINATION
                    aws s3 cp s3://$S3_BUCKET/$S3_OBJECT $SIMULATOR_CONFIG_DESTINATION
                    chmod +x $SIMULATOR_CONFIG_DESTINATION/$S3_OBJECT
                    unzip -qq $SIMULATOR_CONFIG_DESTINATION/$S3_OBJECT -d $SIMULATOR_CONFIG_DESTINATION

                    echo "step: download s3 s3://$S3_BUCKET/$S3_OBJECT into $SIMULATOR_CONFIG_DESTINATION ok" >> /my-cloud-int.log
                    `
                  ].join('\n'),
                  {
                    ignoreErrors: false,
                    key: "0-assets",
                    env: envVariables,
                  }
                ),
                ...commandsFiles.map(item => ec2.InitCommand.shellCommand([
                  `. /opt/simulator-config/assets/commands/${item.file}`
                ].join('\n'), {
                  key: item.name,
                  ignoreErrors: false,
                  env: envVariables,
                })),
              ]
            ),
          }
        });

        cfnInit.attach(carlaInstance, {
          platform: ec2.OperatingSystemType.LINUX,
          instanceRole: carlaInstanceIAMRole,
          userData: userData,
          printLog: true,
          embedFingerprint: true,
          includeRole: true,
          includeUrl: true
        });

        // Add creation policy
        carlaInstance.cfnOptions.creationPolicy = {
          resourceSignal: {
            count: 1,
            timeout: cdk.Duration.minutes(60).toIsoString(),
          }
        };
      }
    }

    // define the biga simulator ec2 instance
    {
      bigaNetworkInterface = new ec2.CfnNetworkInterface(this, `biga-instance-eip`, {
        subnetId: vpc.publicSubnets[1].subnetId,
        description: "Simulator Biga Network Interface",
        groupSet: [vpcSecurityGroup.securityGroupId],
        tags: [{
          key: 'Name',
          value: `${props.resourcePrefix}-biga-instance-eip`,
        }],
      });
      bigaInstance = new ec2.CfnInstance(this, `biga-instance`, {
        tags: [{ key: 'Name', value: `${props.resourcePrefix}-instance-biga` }],
        instanceType: props.bigaInstanceType!,
        imageId: props.bigaMachineImage!.getImage(this).imageId,
        monitoring: true,
        networkInterfaces: [{
          deviceIndex: "0",
          networkInterfaceId: bigaNetworkInterface.ref,
        }],
      });
    }

    // add resources to support unicast or multicast 
    {
      if (props.useMultiCast) {
        // configure the security group for mutlicast 
        vpcSecurityGroup.addIngressRule(ec2.Peer.ipv4(vpc.vpcCidrBlock), ec2.Port.allUdp(), "Multicast - Ingress traffic (UDP)");
        vpcSecurityGroup.addIngressRule(ec2.Peer.ipv4(vpc.vpcCidrBlock), new ec2.Port({
          protocol: ec2.Protocol.IGMP, stringRepresentation: `*`
        }), "Multicast - IGMP Receivers");
        vpcSecurityGroup.addIngressRule(ec2.Peer.ipv4('0.0.0.0/32'), new ec2.Port({
          protocol: ec2.Protocol.IGMP, stringRepresentation: `*`
        }), "Multicast - IGMP Querier");

        // define the transit gateway for multicast
        vpcTransitGateway = new ec2.CfnTransitGateway(this, `transit-gateway`, {
          description: "Multicast Transit Gateway",
          autoAcceptSharedAttachments: "enable",
          defaultRouteTableAssociation: "enable",
          defaultRouteTablePropagation: "enable",
          amazonSideAsn: 64512,
          dnsSupport: "enable",
          multicastSupport: "enable",
          vpnEcmpSupport: "enable",
          tags: [
            {
              key: "Name",
              value: `${props.resourcePrefix}-transit-gateway`
            }
          ]
        });

        vpcTransitGatewayDomain = new ec2.CfnTransitGatewayMulticastDomain(this, `transit-gateway-domain`, {
          transitGatewayId: vpcTransitGateway.attrId,
          options: {
            Igmpv2Support: "enable",
          }
        });

        vpcTransitGatewayAttachment = new ec2.CfnTransitGatewayAttachment(this, `transit-gateway-attachment`, {
          subnetIds: [...Array.from(vpc.publicSubnets, (x) => x.subnetId)],
          transitGatewayId: vpcTransitGateway.attrId,
          vpcId: vpc.vpcId,
        });

        vpcTransitGatewayGroupMemberCarla = new ec2.CfnTransitGatewayMulticastGroupMember(this, `transit-gateway-group-member-carla`, {
          groupIpAddress: "239.255.0.1",
          networkInterfaceId: carlaNetworkInterface.ref,
          transitGatewayMulticastDomainId: vpcTransitGatewayDomain.attrTransitGatewayMulticastDomainId
        });

        vpcTransitGatewayGroupMemberBiga = new ec2.CfnTransitGatewayMulticastGroupMember(this, `transit-gateway-group-member-biga`, {
          groupIpAddress: "239.255.0.1",
          networkInterfaceId: bigaNetworkInterface.ref,
          transitGatewayMulticastDomainId: vpcTransitGatewayDomain.attrTransitGatewayMulticastDomainId
        });

        vpc.publicSubnets.map((subnet, index) => {
          const vpcTransitGatewayDomainAssociation = new ec2.CfnTransitGatewayMulticastDomainAssociation(this, `transit-gateway-domain-association-${index}`, {
            subnetId: subnet.subnetId,
            transitGatewayAttachmentId: vpcTransitGatewayAttachment.attrId,
            transitGatewayMulticastDomainId: vpcTransitGatewayDomain.attrTransitGatewayMulticastDomainId
          });
          vpcTransitGatewayGroupMemberCarla.node.addDependency(vpcTransitGatewayDomainAssociation);
          vpcTransitGatewayGroupMemberBiga.node.addDependency(vpcTransitGatewayDomainAssociation);
        });
      } else {
        // configure the security group for unicast 
        vpcSecurityGroup.addIngressRule(ec2.Peer.ipv4(vpc.vpcCidrBlock), ec2.Port.allUdp(), "Unicast - Ingress traffic (UDP)");
        vpcSecurityGroup.addIngressRule(ec2.Peer.ipv4(vpc.vpcCidrBlock), ec2.Port.udp(3030), "Unicast - Ingress traffic (UDP) on port 3030");

        vpcSecurityGroup.addIngressRule(ec2.Peer.ipv4('0.0.0.0/32'), new ec2.Port({
          protocol: ec2.Protocol.IGMP, stringRepresentation: `*`
        }), "Unicast - IGMP Querier");
      }
    }

    // define the stack outputs
    {
      new cdk.CfnOutput(this, `carla-ami`, {
        key: pascalCase(`${props.resourcePrefix}-carla-ami`),
        exportName: pascalCase(`${props.resourcePrefix}-carla-ami`),
        description: "Carla AMI Id (Ubuntu 20.04 (x86_64))",
        value: `${props.carlaMachineImage!.getImage(this).imageId}`
      });
      new cdk.CfnOutput(this, `biga-ami`, {
        key: pascalCase(`${props.resourcePrefix}-biga-ami`),
        exportName: pascalCase(`${props.resourcePrefix}-biga-ami`),
        description: "Biga AMI Id from CodePipeline for EC2",
        value: `${props.bigaMachineImage!.getImage(this).imageId}`
      });
      new cdk.CfnOutput(this, `carla-instance-type`, {
        key: pascalCase(`${props.resourcePrefix}-carla-instance-type`),
        exportName: pascalCase(`${props.resourcePrefix}-carla-instance-type`),
        description: "Accelerated Instance Type ( https://docs.aws.amazon.com/AWSEC2/latest/UserGuide/accelerated-computing-instances.html )",
        value: props.carlaInstanceType!,
      });
      new cdk.CfnOutput(this, `carla-instance-volume-size`, {
        key: pascalCase(`${props.resourcePrefix}-carla-instance-volume-size`),
        exportName: pascalCase(`${props.resourcePrefix}-carla-instance-volume-size`),
        description: "Volume Size in GiBs (must be equal or larger than snapshot size)",
        value: "40",
      });
      new cdk.CfnOutput(this, `carla-os-user-name`, {
        key: pascalCase(`${props.resourcePrefix}-carla-os-user-name`),
        exportName: pascalCase(`${props.resourcePrefix}-carla-os-user-name`),
        description: "the OS username to be created on the Carla Simulator.",
        value: props.carlaOSUserName!,
      });
      new cdk.CfnOutput(this, `carla-version`, {
        key: pascalCase(`${props.resourcePrefix}-carla-version`),
        exportName: pascalCase(`${props.resourcePrefix}-carla-version`),
        description: "The Carla simulator version to be downloaded and installed",
        value: props.carlaVersion!,
      });
      new cdk.CfnOutput(this, `simulator-allowed-ip`, {
        key: pascalCase(`${props.resourcePrefix}-simulator-allowed-ip`),
        exportName: pascalCase(`${props.resourcePrefix}-simulator-allowed-ip`),
        description: "Allowed IP address to connect to CARLA / Biga via Security Group on port 8443",
        value: props.allowedCIDR,
      });
      new cdk.CfnOutput(this, `simulator-allowed-ip-security-group-id`, {
        key: pascalCase(`${props.resourcePrefix}-simulator-allowed-ip-security-group-id`),
        exportName: pascalCase(`${props.resourcePrefix}-simulator-allowed-ip-security-group-id`),
        description: "The Security group whet the allowed IP address to connect to CARLA / Biga via Security Group on port 8443 is configured",
        value: vpcSecurityGroup.securityGroupId
      });
      new cdk.CfnOutput(this, `simulator-allowed-ip-security-group-url`, {
        key: pascalCase(`${props.resourcePrefix}-simulator-allowed-ip-security-group-url`),
        exportName: pascalCase(`${props.resourcePrefix}-simulator-allowed-ip-security-group-url`),
        description: "The Security group whet the allowed IP address to connect to CARLA / Biga via Security Group on port 8443 is configured",
        value: cdk.Fn.join("", ["https://", props.env?.region!, ".console.aws.amazon.com/ec2/home#SecurityGroup:groupId=", vpcSecurityGroup.securityGroupId])
      });
      new cdk.CfnOutput(this, `biga-instance-type`, {
        key: pascalCase(`${props.resourcePrefix}-biga-instance-type`),
        exportName: pascalCase(`${props.resourcePrefix}-biga-instance-type`),
        description: "Instance type for the biga image",
        value: props.bigaInstanceType!,
      });
      new cdk.CfnOutput(this, `use-multicast`, {
        key: pascalCase(`${props.resourcePrefix}-use-multicast`),
        exportName: pascalCase(`${props.resourcePrefix}-use-multicast`),
        description: "Stack uses Multicast",
        value: String(props.useMultiCast),
      });

      new cdk.CfnOutput(this, `carla-instance-id`, {
        key: pascalCase(`${props.resourcePrefix}-carla-instance-id`),
        exportName: pascalCase(`${props.resourcePrefix}-carla-instance-id`),
        description: "Carla Simulator EC2 Instance ID",
        value: carlaInstance.attrInstanceId,
      });
      new cdk.CfnOutput(this, `carla-instance-id-url`, {
        key: pascalCase(`${props.resourcePrefix}-carla-instance-url`),
        exportName: pascalCase(`${props.resourcePrefix}-carla-instance-url`),
        description: "Carla Simulator EC2 Instance Console link",
        value: cdk.Fn.join("", ["https://", props.env?.region!, ".console.aws.amazon.com/ec2/home#InstanceDetails:instanceId=", carlaInstance.attrInstanceId])
      });
      new cdk.CfnOutput(this, `biga-instance-id`, {
        key: pascalCase(`${props.resourcePrefix}-biga-instance-id`),
        exportName: pascalCase(`${props.resourcePrefix}-biga-instance-id`),
        description: "Biga Simulator EC2 Instance ID",
        value: bigaInstance.attrInstanceId
      });
      new cdk.CfnOutput(this, `biga-instance-url`, {
        key: pascalCase(`${props.resourcePrefix}-biga-instance-url`),
        exportName: pascalCase(`${props.resourcePrefix}-biga-instance-url`),
        description: "Biga Simulator EC2 Instance Console link",
        value: cdk.Fn.join("", ["https://", props.env?.region!, ".console.aws.amazon.com/ec2/home#InstanceDetails:instanceId=", bigaInstance.attrInstanceId])
      });

      new cdk.CfnOutput(this, `carla-instance-public-dns`, {
        key: pascalCase(`${props.resourcePrefix}-carla-instance-public-dns`),
        exportName: pascalCase(`${props.resourcePrefix}-carla-instance-public-dns`),
        description: "Carla Simulator EC2 Instance Public DNS",
        // value: cdk.Fn.join("", [cdk.Fn.getAtt(carlaInstance.node.id, "PublicDnsName").toString()])
        value: carlaInstance.attrPublicDnsName
      });

      new cdk.CfnOutput(this, `carla-secrets-name`, {
        key: pascalCase(`${props.resourcePrefix}-carla-secrets-name`),
        exportName: pascalCase(`${props.resourcePrefix}-carla-secrets-name`),
        description: "Carla Simulator OS user & password secret ARN",
        value: carlaSecret.secretName
      });
      new cdk.CfnOutput(this, `carla-secrets-url`, {
        key: pascalCase(`${props.resourcePrefix}-carla-secrets-url`),
        exportName: pascalCase(`${props.resourcePrefix}-carla-secrets-url`),
        description: "Carla Simulator OS user & password secret ARN",
        value: cdk.Fn.join("", ["https://", props.env?.region!, ".console.aws.amazon.com/secretsmanager/secret?name=", carlaSecret.secretName])
      });

      new cdk.CfnOutput(this, `carla-ssm-session-manager-url`, {
        key: pascalCase(`${props.resourcePrefix}-carla-ssm-session-manager-url`),
        exportName: pascalCase(`${props.resourcePrefix}-carla-ssm-session-manager-url`),
        description: "SSM Session Manager access link for Carla (use Session Manager)",
        value: cdk.Fn.join("", ["https://", props.env?.region!, ".console.aws.amazon.com/systems-manager/session-manager/", carlaInstance.attrInstanceId])
      });
      new cdk.CfnOutput(this, `biga-ssm-session-manager-url`, {
        key: pascalCase(`${props.resourcePrefix}-biga-ssm-session-manager-url`),
        exportName: pascalCase(`${props.resourcePrefix}-biga-ssm-session-manager-url`),
        description: "SSM Session Manager access link for Biga (use EC2 serial console)",
        value: cdk.Fn.join("", ["https://", props.env?.region!, ".console.aws.amazon.com/systems-manager/session-manager/", bigaInstance.attrInstanceId])
      });

      new cdk.CfnOutput(this, `nice-dcv-client-web-console-url`, {
        key: pascalCase(`${props.resourcePrefix}-nice-dcv-client-web-console-url`),
        exportName: pascalCase(`${props.resourcePrefix}-nice-dcv-client-web-console-url`),
        description: "NICE DCV Web Access Console URL",
        // value: cdk.Fn.join("", ["https://", cdk.Fn.getAtt(bigaInstance.logicalId, "PublicDnsName").toString(), ":8443"])
        value: cdk.Fn.join("", ["https://", carlaInstance.attrPublicDnsName, ":8443"])
      });
      new cdk.CfnOutput(this, `nice-dcv-client-connection-string1`, {
        key: pascalCase(`${props.resourcePrefix}-nice-dcv-client-connection-string`),
        exportName: pascalCase(`${props.resourcePrefix}-nice-dcv-client-connection-string`),
        description: "NICE DCV Client Connection String",
        // value: cdk.Fn.join("", [cdk.Fn.getAtt(bigaInstance.logicalId, "PublicDnsName").toString(), ":8443"])
        value: cdk.Fn.join("", [carlaInstance.attrPublicDnsName, ":8443"])
      });

      new cdk.CfnOutput(this, `nice-dcv-download-url`, {
        key: pascalCase(`${props.resourcePrefix}-nice-dcv-download-url`),
        exportName: pascalCase(`${props.resourcePrefix}-nice-dcv-download-url`),
        description: "NICE DCV client download URL link",
        value: "https://download.nice-dcv.com"
      });
      new cdk.CfnOutput(this, `carla-local-path`, {
        key: pascalCase(`${props.resourcePrefix}-carla-local-path`),
        exportName: pascalCase(`${props.resourcePrefix}-carla-local-path`),
        description: "CARLA installation target path",
        value: "/opt/carla-simulator/"
      });
      new cdk.CfnOutput(this, `carla-config-local-path`, {
        key: pascalCase(`${props.resourcePrefix}-carla-config-local-path`),
        exportName: pascalCase(`${props.resourcePrefix}-carla-config-local-path`),
        description: "CARLA Simulator Config target path",
        value: SIMULATOR_CONFIG_DESTINATION
      });
    }
  }
}
