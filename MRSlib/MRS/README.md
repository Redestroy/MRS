# MRSlib

## MRSlib descriptions

## MRSlib sections
- MRS: core functionality, contains atomics such as Action, View and Condition, as well as system data structures
- MRS/Device: 2nd layer system functionality, providing device base classes and interfaces, backbone of the System functionality.
- MRS/Data: Side layer system functionality, providing the necessary view classes, models and worldviews, key in the visualizer shell operation.
- MRS/Task: 3rd layer object functionality, providing Task classes and generators.
- MRS/Algorithms: 3rd layer functional implementation, using tasks as guidance for robot operation.
- MRS/Coms: Inter layer communication library for relaying information between layers.

External libraries:
- MRS/Overseer: provides algorithms for multi system control. 