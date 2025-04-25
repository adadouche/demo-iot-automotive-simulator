#!/usr/bin/env node

// import { exec } from 'child_process';
import * as cdk from 'aws-cdk-lib';
import * as ec2 from 'aws-cdk-lib/aws-ec2';
import { BigaSimulatorStack } from '../lib/biga-simulator-stack';

const { execSync } = require('child_process');
const cmd = `curl -s http://checkip.amazonaws.com || printf "0.0.0.0"`;
const currentIP = `${execSync(cmd).toString().trim()}/32`;

const resourcePrefix = "biga-simulator";

const app = new cdk.App();

// check if the useMultiCast flag is set
var useMultiCast: boolean = true;
if (app.node.tryGetContext('useMultiCast') == undefined) {
  console.log("Defaulting to useMultiCast = true. To disable multicast, add '-c useMultiCast=false' to your cdk deploy command.");
} else if (app.node.tryGetContext('useMultiCast') === 'true') {
  useMultiCast = true;
} else if (app.node.tryGetContext('useMultiCast') === 'false') {
  useMultiCast = false;
} else {
  useMultiCast = true;
}

/* See https://docs.aws.amazon.com/sdkref/latest/guide/access.html for details on how to access AWS. */
const env = {
  account: process.env.CDK_DEFAULT_ACCOUNT || process.env.AWS_DEFAULT_ACCOUNT,
  region: process.env.CDK_DEFAULT_REGION || process.env.AWS_DEFAULT_REGION,
};

/**
 * Use these default props to enable termination protection and tag related AWS
 * Resources for tracking purposes.
 */
const defaultProps: cdk.StackProps = {
  terminationProtection: false, // TODO: enable or remove.
  env,
};

const carlaMachineImage = ec2.MachineImage.fromSsmParameter(
  `/aws/service/canonical/ubuntu/server/20.04/stable/current/amd64/hvm/ebs-gp2/ami-id`,
  {
    os: ec2.OperatingSystemType.LINUX,
  }
);

const bigaMachineImage = ec2.MachineImage.lookup({
  name: 'aws-biga-image-aws-ec2-arm64*',
  filters: {
    "tag:Name": ['biga-ci-pipeline-ec2'],
  },
  owners: ['self'],
  windows: false,
});

new BigaSimulatorStack(
  app,
  `${resourcePrefix}`,
  {
    ...defaultProps,
    description: "AWS IoT Automotive Demo - Biga Simulator Stack",
    resourcePrefix: resourcePrefix,
    allowedIPs: `${currentIP}`,
    bigaMachineImage: bigaMachineImage,
    carlaMachineImage: carlaMachineImage,
    useMultiCast: useMultiCast,

    // carlaInstanceType: "t3.xlarge",
    carlaInstanceType: "g5.2xlarge",
    repositoryURL: "https://github.com/adadouche/demo-iot-automotive-simulator",
  }
);