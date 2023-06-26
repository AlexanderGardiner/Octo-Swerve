<h1>The development robot code for Octo-Lib</h1>
<h3><i>Made by team 9084.</i></h3>
<h2>For development use as a normal robot project</h2>
<h2>How to update the library</h2>

1. Open the octo-lib project

2. Copy the folders that are going to be put into the library

3. Paste them in lib/src/main/java/octo/lib in the octo-lib project

4. Change the version number in the build.gradle file in the octo-lib project under wpi
   <br><code>wpi {
    &nbsp;&nbsp;&nbsp;&nbsp;version = "2023.1.0"
   }</code>

5. Build the octo-lib project

6. The library .jar file will be in lib/build/libs in the octo-lib project and called octo-lib-*version*.jar (The .jar file will not be pushed to github)

7. Create a new github release with the .jar file in the octo-lib repository

