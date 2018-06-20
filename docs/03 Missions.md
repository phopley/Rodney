# Robotic Missions
In order to come up with the requirements for the project I'm going to specify some "missions" that I would like Rodney to be able to perform.
## Mission 1 - Take a message to...
In the article [Let's build a robot!](https://www.codeproject.com/Articles/1115414/Lets-build-a-robot "Let's build a robot!") referenced in the introduction, the author of the article lists jobs he would like the robot to do around the house. One of these jobs is:

"Take a message to... - Since the robot will [have] the ability to recognize family members, how about the ability to make it the 'message taker and reminder'. I could say 'Robot, remind (PersonName) to pick me up from the station at 6pm'. Then, even if that household member had their phone turned on silent, or were listening to loud music or (insert reason to NOT pick me up at the station), the robot could wander through the house, find the person, and give them the message."

This sounds like a good starting point for the first mission. I'm going to change it slightly though, what if I could access a server that stored a list of missions and the server passed on these missions to the robot.

Now I'm going to breakdown the "Take a message to..." mission into several smaller design goals that can be worked on and completed on their own. The design goals for this mission will be:
- Design Goal 1: To be able to look around using the camera, search for faces, attempt to identify any seen and display a message for any one identified
- Design Goal 2: Facial expressions and speech synthesis
- Design Goal 3: Locomotion controlled by Teleop (Remote control from a keyboard and/or joystick)
- Design Goal 4: Addition of a laser ranger finder or similar ranging sensor
- Design Goal 5: Autonomous locomotion
- Design Goal 6: Task assignment and completion notification

That's quite a list of things to accomplish for what seems like a simple mission as first.
