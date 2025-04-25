import canigen
import time

c = canigen.canigen(
    interface='vcan0',
    database_filename='chevy.dbc',
    obd_config_filename='obd_config_chevy.json',
    values_filename='vals_chevy_default.json')

print('sleeping')
time.sleep(1.0)

for i in range(0, 1000, 50):
    print('Engine_Torque='+str(i))
    c.set_sig('Engine_Torque', i)
    time.sleep(0.1)
print('sleeping')
time.sleep(0.5)

for i in range(0, 1000, 50):
    print('ENGINE_SPEED='+str(i))
    c.set_pid('ENGINE_SPEED', i)
    time.sleep(0.1)
print('sleeping')
time.sleep(0.5)

print('set dtc ECM_DTC1 failed')
c.set_dtc('ECM_DTC1', 1)
time.sleep(2.0)

print('set dtc ECM_DTC1 cleared')
c.set_dtc('ECM_DTC1', 0)
time.sleep(1.0)

print('stopping')
c.stop()
