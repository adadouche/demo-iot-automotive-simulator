#!/bin/bash

# Parsing command-line arguments
additional_carla_client_args="$@"

log_dir="./logs"
mkdir -p $log_dir

# Function to log messages with timestamp
log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$log_dir/main.log"  2> /dev/null
}

# Function to start a process
start_process() {
    local command=$1
    local log_file=$2
    # log "Runnig $command - Logging to $log_file"
    setsid $command > $log_file 2>&1 &
    echo $! # Return the PID
}

killit() {
    search_string=$1
    # log "killing ${search_string} process..."
    kill -9 $(ps aux | grep "${search_string}" | awk '{print $2}') > /dev/null 2>&1
}

# Function to kill all process groups
cleanup() {
    if [ ! "$cleanup_in_progress" = true ] ; then
        cleanup_in_progress=true
        for pid in "${pids[@]}"; do
            # log "killing pid ${pid} ..."
            kill -9 $pid > /dev/null 2>&1 # Kills the entire process group
        done
        pkill CarlaUE4
        killit 'multicast_can_send.sh'
        killit '/opt/carla-simulator/CarlaUE4'
        killit 'python3 ./manual_control_steeringwheel.py'
        killit 'gpython3 /opt/carla-simulator/PythonAPI/examples/generate_traffic.py'
        killit 'ros2 launch carla_ros_bridge carla_ros_bridge.launch.py'
        killit 'ros2 launch carla_spawn_objects carla_spawn_objects.launch.py'
        killit 'python3 ./image_converter.py'
        cleanup_in_progress=false
    fi
    if [ "$exit_after_cleanup" = true ] ; then
        exit
    fi    
}

# Trap script termination signals to cleanup
trap cleanup EXIT SIGINT SIGTERM

# Main execution
main() {
    # cleanup everything first
    cleanup

    exit_after_cleanup=true
    # Array to store process group IDs
    declare -a pids

    if [[ "$additional_carla_client_args" == *"vcan0"* ]]; then
        log "start vcan multicast forwarding"
        pid=$(start_process "./multicast_can_send.sh" "$log_dir/socketcan.log")
        pids+=($pid)
        log "  -> process pid   : $pid"
    fi

    # Start processes
    log "starting CarlaUE4"
    pid=$(start_process "/opt/carla-simulator/CarlaUE4.sh -no-rendering -quality-level=Low -prefernvidia" "$log_dir/carla.log")
    pids+=($pid)
    log "  -> process pid   : $pid"
    # log "  -> monitored pid : ${pids[*]}"
    sleep 15
    
    source /opt/ros/galactic/setup.bash
    source ~/ros2_ws/install/setup.bash

    cd carla-client
    log "starting manual_control_steeringwheel"
    # pid=$(start_process "python3 ./manual_control_steeringwheel.py --sync --rolename ego_vehicle --filter vehicle.tesla.model3 $additional_carla_client_args" "../$log_dir/control.log")
    pid=$(start_process "python3 ./manual_control_steeringwheel.py --sync --rolename ego_vehicle $additional_carla_client_args" "../$log_dir/control.log")
    pids+=($pid)
    log "  -> process pid   : $pid"
    # log "  -> monitored pid : ${pids[*]}"
 
    log "starting generate_traffic"
    pid=$(start_process "python3 /opt/carla-simulator/PythonAPI/examples/generate_traffic.py -n 15 -w 20" "../$log_dir/traffic_gen.log")
    pids+=($pid)
    log "  -> process pid   : $pid"
    # log "  -> monitored pid : ${pids[*]}"
    
    cd ..

    sleep 10

    log "starting carla_ros_bridge"
    pid=$(start_process "ros2 launch carla_ros_bridge carla_ros_bridge.launch.py timeout:=20000 register_all_sensors:=false synchronous_mode:=false passive:=true" "$log_dir/carla_ros_bridge.log")
    pids+=($pid)
    log "  -> process pid   : $pid"
    # log "  -> monitored pid : ${pids[*]}"


    log "starting carla_spawn_objects"
    pid=$(start_process "ros2 launch carla_spawn_objects carla_spawn_objects.launch.py spawn_sensors_only:=True objects_definition_file:=./carla-client/ros2/objects.json" "$log_dir/carla_spawn_objects.log")
    pids+=($pid)
    log "  -> process pid   : $pid"
    # log "  -> monitored pid : ${pids[*]}"

    cd carla-client/ros2

    log "starting image_converter rgb_front"
    pid=$(start_process "python3 ./image_converter.py --input_topic /carla/ego_vehicle/rgb_front/image --output_topic /carla/ego_vehicle/rgb_front/compressed_image" "../../$log_dir/image_conv1.log")
    pids+=($pid)
    log "  -> process pid   : $pid"
    # log "  -> monitored pid : ${pids[*]}"
    
    log "starting image_converter depth_front"
    pid=$(start_process "python3 ./image_converter.py --input_topic /carla/ego_vehicle/depth_front/image --output_topic /carla/ego_vehicle/depth_front/compressed_image" "../../$log_dir/image_conv2.log")
    pids+=($pid)
    log "  -> process pid   : $pid"
    # log "  -> monitored pid : ${pids[*]}"
    
    cd ../../

    # log "starting image_converter depth_front"
    # pid=$(start_process "sudo python3 ./observability/can-stats/generate_stats.py" "./$log_dir/generate_stats.log")
    # pids+=($pid)
    # log "  -> process pid   : $pid"
    # # log "  -> monitored pid : ${pids[*]}"
    
    log "activating monitoring on pid every 5 secs: ${pids[*]}"

    # Monitor all process groups
    while : ; do
        for pid in "${pids[@]}"; do
            if ! ps -p $pid > /dev/null 2>&1; then
                log "Process $pid has terminated unexpectedly. Exiting completely."
                # cleanup stop
                exit 1
            fi
        done
        sleep 5
    done
}

# Run the main function
main
