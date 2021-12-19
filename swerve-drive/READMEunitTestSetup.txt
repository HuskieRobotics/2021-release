~~~~~~~~~~~~~~~~~~~~~~~~~~~Tutorial on how to set up a regular java robot project for unit testing~~~~~~~~~~~~~~~~~~~~~~~~~~~

Step 1
Paste this code in build.gradle so build.gradle knows to install the mockito dependency from MavenCentral:
repositories {
    mavenCentral()
}

Step 2
Paste this code inside the {}s of build.gradle dependencies to tell the gradle to compile the mockito dependency with version 2.23 (it is important to use this version since this version supports Java 11):
testCompile group: 'org.mockito', name: 'mockito-core', version: '2.23.0'

Step 3
Create a folder named "test\java\frc\robot" inside the src folder
If your robot project is a command project, add the "commands" & "subsystems" folders inside the test\java\frc\robot folder

Step 4
Add this to the .classpath file so VSCode recognizes the path your Unit Tests (.classpath file may not show up in VSCode, .classpath file should show up next to your build.gradle file in VSCode):
<classpathentry kind="src" output="bin/test" path="src/test/java">
    <attributes>
        <attribute name="gradle_scope" value="test"/>
        <attribute name="gradle_used_by_scope" value="test"/>
        <attribute name="test" value="true"/>
    </attributes>
</classpathentry>

Step 5
delete the ".classpath" line from the .gitignore so Github Desktop knows to commit your .classpath file's change to your branch/repository

Step 6
Paste this as a new setting in the settings.json file inside the .vscode folder. This tells FRC VSCode where to find the JNI libaries needed for many WPI Modules (remember to put a comma between any previous settings and this one!):
"java.test.config": [
    {
        "name": "WPIlibUnitTests",
        "workingDirectory": "${workspaceFolder}/build/tmp/jniExtractDir",
        "vmargs": [ "-Djava.library.path=${workspaceFolder}/build/tmp/jniExtractDir" ],
        "env": { "LD_LIBRARY_PATH": "${workspaceFolder}/build/tmp/jniExtractDir" ,
          "DYLD_LIBRARY_PATH": "${workspaceFolder}/build/tmp/jniExtractDir" }
        
    },
  ],
  "java.test.defaultConfig": "WPIlibUnitTests"

Step 7
Ctrl+Shift+P to open command palette
Choose this command:
run Java: clean Java Language server workspace

Step 8
install the Java Test Runner extension in your FRC VSCode

Step 9
Close out of FRC VSCode and reopen it. After that, you're done!

All unit tests go in src\test\java\frc\robot, and follow the same file structure & package statements as regular robot code
Wpilib & CTRE imports also seem to work in unit tests if settings.json was set up correctly


~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Basic structure of an FRC Unit Test~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
package frc.robot; //package frc.robot.subsystems if in subsystems folder, package frc.robot.commands if in commands folder

import org.junit.*;
import static org.junit.Assert.*;
import org.mockito.*;
import static org.mockito.Mockito.*;

public class TemplateUnitTest {
    @Before
    public void setup() {
        // code that runs before every test
    }
    @Mock
    // put your mock instance variables here
    Object objectMock;
    @Test
    public void myUnitTest() {
        //write code for your unit test here
    }
    @Test
    public void myUnitTest2() {
        //write code for your 2nd unit test here
    }
}

~~~~~~~~~~~~~~~~~~~~~~~~~~~How to run an FRC Unit Test~~~~~~~~~~~~~~~~~~~~~~~~~~~
Make sure that you have the Java Test Runner extension installed
Click on the "run test" button that appears above a unit test method and below the @Test annotation
Example:

@Test
Run Test | Debug Test
public void myUnitTest() {
    //write code for your unit test here
}

^^^^^^^^Unit test should look like this^^^^^^^^^^^^^
If unit test passed, a checkmark should pop up next to the "Run Test | Debug Test" buttons. Click on the checkmark to view details about the test in a new window
Example:
@Test
Run Test | Debug Test ✓
public void myUnitTest() {
    //write code for your unit test here
}
If unit test failed, an X should appear in the same spot. Click on the X to view details about the test in a new window

~~~~~~~~~~~~~~~~~Known Bugs~~~~~~~~~~~~~~~~~
Cannot find jniExtractDir because it does not exist:
Make sure .scode\settings.json has correct settings (see above to setup settings.json correctly)
Right click on build.gradle, click on "build robot code".