Hello and welcome to team 5921, or the La Canada Engineering Club's repository for the 2022-2023 FTC season powerplay!
If the current season is still Powerplay, please talk to Luke or Leo because we need more programmers. If it's a future season,
also talk to Luke or Leo if you have any questions about anything.
Some questions that I had when starting out, with answers:

Q: Where are the actual programs that get run? 
A: Teamcode/src/main/java/org/firstinspires/ftc/teamcode, take a look at the readme there

Q: What's an opmode? 
A: Basically, a program for the robot to run.

Q: Where is the actual code that gets run, or the "main" method? 
A: It doesn't really matter, just put your stuff in a file in teamcode.

Q: Where are the motors and things defined? 
A: For this codebase, *Teamcode/src/main/java/org/firstinspires/ftc/teamcode/common/HardwareDrive.java*

Q: How do I run a program? 
A: Make sure the program you want to run extends the LinearOpMode abstract, and also make sure it's annotated with `@Autonomous` or `@TeleOp`, and then run it from the driver station. Check out *Teamcode/src/main/java/org/firstinspires/ftc/teamcode/teleop/BaseDriveComplete*
