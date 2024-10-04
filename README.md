# Line-Follower-Robot-using-advanced-menu-controlled-system
OLED display, arrduino MEGA,TB66F16G




<h1 style="text-align: center; color: #2e6da4;">Arduino Line Following Robot with OLED Menu System</h1>

<p style="text-align: center;">
  <img src="https://github.com/user-attachments/assets/68cdde05-c410-4105-81c4-16e43a822800" alt="Line Following Robot" width="300" />
    <img src="https://github.com/user-attachments/assets/84d72e9b-948f-4685-b494-f2476a3a65eb" width="280" />
</p>



<p>Welcome to the Arduino Line Following Robot project! This robot uses an Arduino microcontroller to follow lines on a track, with a sophisticated menu system displayed on an OLED screen for easy parameter adjustments.</p>

<hr>

<h2 style="color: #2e6da4;">🚀 Features</h2>
<ul>
  <li>🔄 <strong>Line Following:</strong> Accurate line tracking using multiple sensors.</li>
  <li>🎛️ <strong>PID Control:</strong> Fine-tuned for smooth and precise movement.</li>
  <li>📟 <strong>OLED Menu:</strong> Navigate through options and settings with ease.</li>
  <li>🎮 <strong>Push Button Navigation:</strong> Three-button interface for menu control.</li>
  <li>⚙️ <strong>Adjustable Motor Speeds:</strong> Customize speed settings to optimize performance.</li>
  <li>📏 <strong>Sensor Calibration:</strong> Adapt to different track conditions.</li>
</ul>
<h2>🔧 Hardware Requirements</h2>
<ul>
  <li>Arduino Board (e.g., Arduino Mega Pro Mini) 🧑‍🔧</li>
  <li>8 Line Sensors 🎯</li>
  <li>Motor Driver (e.g., TB6612FNG) 🛠️</li>
  <li>Two DC Motors ⚡</li>
  <li>OLED Display (SSD1306 128x32) 📺</li>
  <li>Three Pushbuttons 🔘</li>
  <li>Battery (e.g., 2 cell 18650 lithium-ion batteries) 🔋</li>
</ul>

<hr>

<h2 style="color: #2e6da4;">📚 Libraries Used</h2>
<ul>
  <li><code>U8glib</code> - For controlling the OLED display.</li>
  <li><code>Pushbutton</code> - For easy button handling.</li>
</ul>

<hr>

<h2 style="color: #2e6da4;">📋 Menu System</h2>
<p>The robot features a dynamic OLED menu system that you can navigate using three buttons:</p>
<ul>
  <li>🔼 <strong>Button 1:</strong> Move up in the menu.</li>
  <li>🔘 <strong>Button 2:</strong> Select an option or enter a submenu.</li>
  <li>🔽 <strong>Button 3:</strong> Move down in the menu.</li>
</ul>


<p>Main menu options include:</p>
<ul>
      <img src="https://github.com/user-attachments/assets/6370b107-4d1a-4115-95b5-35bef3f11053" width="280" />
   <img src="https://github.com/user-attachments/assets/1aacf89d-4810-44d6-8749-df6f7e6b212d" width="295" />
  <li>⚙️ <strong>Calibration:</strong> Starts the line sensor calibration process.</li>
  <li>🔍 <strong>Sensor Test:</strong> Checks the line sensors and displays results on the OLED screen.</li>
  <li>➡️ <strong>Follow Line:</strong> Enables the line-following mode.</li>
  <li>🏎️ <strong>RUN:</strong> Enters the motor control submenu.</li>
</ul>

<hr>


<h2 style="color: #2e6da4;">🛠️ Motor Control Submenu</h2>
<p>In the motor control submenu, you can:</p>
<ul>
        <img src="https://github.com/user-attachments/assets/2673af4a-d606-47c6-aaaa-decc98de179f" width="280" />
  <li>🟢 <strong>Toggle Motor:</strong> Turn the motor on or off.</li>
  <li>⚡ <strong>Adjust LFSPEED:</strong> Fine-tune the line-following speed.</li>
  <li>🔥 <strong>Adjust MAXSPEED:</strong> Set the maximum speed for the motors.</li>
  <li>↩️ <strong>Return to Menu:</strong> Go back to the main menu.</li>
</ul>

<hr>

<h2 style="color: #2e6da4;">🧩 Code Overview</h2>
<p>The code is structured into several key sections:</p>
<ul>
  <li><strong>Setup:</strong> Initializes the OLED display, buttons, and motor control pins.</li>
  <li><strong>Loop:</strong> Handles menu navigation, mode execution, and motor control.</li>
  <li><strong>Menu Functions:</strong> Manages the drawing and updating of the menu.</li>
  <li><strong>Motor Control:</strong> Controls motor speed and direction using PWM signals.</li>
  <li><strong>Calibration:</strong> Calibrates the line sensors for optimal tracking.</li>
</ul>

<hr>

<h2 style="color: #2e6da4;">📖 How to Use</h2>
<ol>
  <li>Download and install the necessary libraries in the Arduino IDE.</li>
  <li>Upload the code to your Arduino board.</li>
  <li>Power the robot and observe the OLED display for the menu.</li>
  <li>Use the buttons to navigate through the menu and select the desired mode.</li>
  <li>Calibrate the sensors if necessary, then start the line-following mode.</li>
  <li>Adjust the motor speeds as needed for optimal performance.</li>
</ol>

<hr>

<h2 style="color: #2e6da4;">📦 Dependencies</h2>
<ul>
  <li>Arduino IDE</li>
  <li>U8glib library for OLED display</li>
  <li>Pushbutton library for button handling</li>
</ul>

<hr>
<h2 style="color: #2e6da4;">📦 Acknowledgments</h2>
<ul>
   <li>Inspired by various open-source line-following robot projects.</li>
  <li>Special thanks to the authors of the U8glib and Pushbutton libraries for their contributions.</li>
</ul>
<hr>

<h2 style="color: #2e6da4;">📜 License</h2>
<p>This project is open-source and licensed under the <a href="https://opensource.org/licenses/MIT">MIT License</a>.</p>
<hr>

<h2 style="color: #2e6da4;">📧 Contact</h2>
<p>If you have any questions, suggestions, or would like to get the full project code, feel free to reach out to me via email at <a href="mailto:almasrafeshahria1234@gmail.com" style="color: #2e6da4;">gmail</a>.</p>

<hr>
<hr>

<p style="text-align: center;">
  <img src="https://github.com/user-attachments/assets/ff276992-ced4-4110-a034-e6a0c70d3fca" width="1000" />
</p>





