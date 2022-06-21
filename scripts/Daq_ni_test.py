import nidaqmx
import nidaqmx.system



system = nidaqmx.system.System.local()
system.driver_version


for device in system.devices:
	print(device)

	
	
'''
with nidaqmx.Task() as task:
	task.ai_channels.add_ai_voltage_chan("Dev3/ai0")
	task.read()
'''	
	
