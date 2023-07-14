<h1>The development robot code for Octo-Lib</h1>
<h3><i>Made by team 9084.</i></h3>
<h2>For development use as a normal robot project</h2>
<h2>How to update the library</h2>

1. Open the octo-lib project

2. Copy the folders that are going to be put into the library

3. Paste them in lib/src/main/java/octo/lib in the octo-lib project

4. Press <kbd>ctrl</kbd>+<kbd>shift</kbd>+<kbd>f</kbd> to find all instances of "frc.robot.Libraries" in the project

6. Then press the arrow on the left of the search field and in the field below the search field (the replace field) type "octo.lib"

7. Press the button to the right of the replace field to replace all instances of "frc.robot.Libraries" with "octo.lib"

9. Change the version number in the build.gradle file in the octo-lib project under wpi (If it asks to synchronize the classpath/configuration press yes)
   <br><code>wpi {
    &nbsp;&nbsp;&nbsp;&nbsp;version = "2023.1.0"
   }</code>

10. Build the octo-lib project

11. The library .jar file will be in lib/build/libs in the octo-lib project and called octo-lib-*version*.jar (The .jar file will not be pushed to github)

12. Create a new github release with the .jar file in the octo-lib repository

