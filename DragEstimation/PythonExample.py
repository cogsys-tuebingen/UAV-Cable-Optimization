from DragEstimation import DragEstimation
from Visualization import visualize_input, visualize_drag

drag = DragEstimation()
drag.load("ExampleDataflashLog.bin")
drag.process_data(start=0.25, end=0.9, pitch_offset=1.4955439766569403)
result = drag.fit(drone_mass=1.549)
print("Front Area, Top Area, Linear Term, Wind Offset")
print(result)
visualize_input(drag)
visualize_drag(drag)
