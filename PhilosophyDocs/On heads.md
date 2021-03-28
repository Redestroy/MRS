# On robot heads and their necessity

A head device is not just the root node of the robot. A head device is the device that controls the robot on the top layer from the robots view. This means that a robot without a head should not exist, right?

  Well, if we look at swarm robots, they are single device entities, autonomous, yet also not. If we assume the necessity of the head, it follows that a swarm needs a controller - something that has control over all swarm bots. If wireless communication is assumed, it can be viewed as a centralised system. In such a view scalability is lost tho', since each controller needs to maintain links with its bots. This leads to the necessity of variable device structures - device trees are insufficient. Such a bot is between a regular robot and a device - it has the bearest amount of autonomy, yet to achieve the completion of tasks and gain usefullness, they require a controller. A Swarm robot can therefore be a robot that is made of multiple bodies but only a single head.

  A device tree is the foundation of regular robots. It has actuators and sensors as leaf nodes,
joints as body nodes and finally a single head node as root. Such a structure is sufficient in absolute control of the robot. Some toughts might come to one head rule for a device tree - the truth is it is arbitrary. Based on a use case, more than one head might be usefull in certain scenarious, or it could be possible to have no heads as in with swarm robots. This leads us to the true definition of a head node - head node is the node of responsibility. If a robot has no head node, it cannot be a true robot for it does not have autonomy within it. If a robot has a single head node it decides how to allocate its resources and use its devices.

  This leads as to Multi head robots - hydras. While there is a saying "two heads are better than one", Multi-head paradigm puts this to the test. If we abbandon the physicality of the head node, as we have with integral compound nodes, it could be argued that multiple agent programs running simultaniously on a single physical head device can be viewed as multiple head robot - a hydra. Hydras might be usefull for planning purposes - using multiple gready aproaches on the state space to execute a star search based planning. Afterwords hydra votes or evaluates the usefullness of the plans of each head and the head whose plan wins, takes control of the body. Another possible use is for the execution of the Multi-task task. Each head allocates its resources by using distributed computing techniques and takes on one task at a time. Thus, a hydra could in theory solve the MRMT problem in a neat way. Aside from these two cases, hydras are two complex to be used in regular robot systems.

Neo robot systems are the final step of robotics - afterwards, the boundry of the robot gets so unclear that it is not usfull as a concept. Neo robot systems are superior to swarm robots, regular robots and hydras as they both have multiple bodies as swarm robots, multiple heads as hydras and use a dynamic device structure that is a fusion between multi agent paradigm and distributed computing (might be similar to artificial life). These robots use IOT similar concepts to extend the boundry of the robot to the size of the system. Such systems ar still far away, however they could, in theory, provide the most adaptable kind of robot systems - such systems might even be able to physically self modify and self replicate.

Recap:
True robot - robot with at least one head node
Regular robot - robot that can be represented with a device tree
Hydra - multi headed robot
Neo robot systems - MRS system that is self reconfigurable to the highest degree - n bodies and n heads with a dynamic structure.

Note on cloud hydras -
Technically it is possible to use a cloud to transport agent programms between robots. This could be usefull for future to improve the robots through GA, GP and ML techniques (and probably a prerequisite for neo robot systems)
