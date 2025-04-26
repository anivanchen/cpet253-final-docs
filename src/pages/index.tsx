import Image from "next/image";
import { Geist, Geist_Mono } from "next/font/google";

const geistSans = Geist({
  variable: "--font-geist-sans",
  subsets: ["latin"],
});

const geistMono = Geist_Mono({
  variable: "--font-geist-mono",
  subsets: ["latin"],
});

export default function Home() {
  return (
    <div className="grid items-center justify-items-center min-h-screen p-8 font-[family-name:var(--ont-geist-sans)]">
      <main className="flex flex-col items-center max-w-[1440px]">
        <h1 className="mt-[4rem] mb-[2rem] text-[2rem] font-bold">Two Wheel Odometry</h1>
        <p className="mb-[0.25rem]">Ivan Chen</p>
        <p className="mt-[0.25rem] mb-[4rem]">This project was done as the final project of CPET-253 (Microcontroller Systems) at RIT.</p>
        <div className="grid grid-cols-1 md:grid-cols-[30%_70%] mb-[2rem] items-center">
          <div className="relative mx-auto rounded-lg overflow-hidden">
            <Image className="mt-[2rem] mb-[1rem] ml-[4rem] mr-[4rem] " src="/splash-image.jpg" width={400} height={500} alt="Picture of the robot"/>
          </div>
          <div>
            <h3 className="mt-[2rem] mb-[1rem] ml-[4rem] mr-[4rem] text-[1.2rem] font-bold">Project Description</h3>
            <p className="mt-[1rem] mb-[2rem] ml-[4rem] mr-[4rem]">This project aims to provide the distance the robot's left and right wheels have traveled using rotary Hall Effect quadrature encoders. This lets us track the position and orientation of the robot over time, known as odometry, without an Inertial Measurement Unit (IMU). We will use this information about the robot's position to drive the robot towards its starting point (a "homing function"). The robot will also be able to wander around a room while avoiding obstacles using ultrasonic sensors, all while continuously tracking its position (a "wandering function"). The robot will be controlled using a mobile smartphone, which will communicate with the robot using Bluetooth. Two finite state machines control the mode of the robot, and the state of its motors.</p>
          </div>
          
          <div>
            <p>INSERT IMAGES</p>
          </div>
          <div>
            <h3 className="mt-[2rem] mb-[1rem] ml-[4rem] mr-[4rem] text-[1.2rem] font-bold">Encoders</h3>
            <p className="justify-start mt-[1rem] mb-[1rem] ml-[4rem] mr-[4rem]">Hall effect quadrature encoders work by using a rotating disc with evenly spaced magnets. As the disc rotates, these magnets pass over two Hall effect sensors, which are positioned slightly apart. Each sensor outputs a signal that switches between high and low states as the magnets pass by. Because the sensors are offset, their signals are out of phase, creating two pulse lines (A and B) offset by 90 degrees. By monitoring the sequence of these pulses, we can determine the direction of rotation. The number of pulses also tells us how far the wheel has rotated.</p>
            <p className="justify-start mt-[1rem] mb-[1rem] ml-[4rem] mr-[4rem]">To capture these pulses, we use hardware interrupts that read both lines when triggered, and this figures out the direction the motor is being driven in. Given that A is BIT1 and B is BIT0, reading <code className="mt-[1rem] mb-[1rem] bg-black/[.05] dark:bg-white/[.06] px-[6px] py-[3px] rounded font-semibold">01</code> would mean going in one direction, and reading <code className="mt-[1rem] mb-[1rem] bg-black/[.05] dark:bg-white/[.06] px-[6px] py-[3px] rounded font-semibold">10</code> would mean going in the opposite direction. We have a left and right encoder, so we must keep track of two encoders counts.</p>
            <p className="justify-start mt-[1rem] mb-[1rem] ml-[4rem] mr-[4rem]">In this case, the wheels had a diameter of 2.75 inches, a gear ratio of 60:1 between the motor shaft and wheel shaft and 12 counts per motor shaft rotation. This means that to convert actual distance traveled (in feet) from raw encoder counts, which is counted on both the rising and falling edges, we must do a unit conversion. This unit conversion is </p>
            
            <pre className="bg-black/[.05] dark:bg-white/[.06] mx-[4rem] px-[1rem] py-[1rem] rounded font-[family-name:var(--font-geist-mono)] font-semibold text-wrap">
              <code>
                dist_in_ft = (counts / (60.0 * 12.0)) * 2.0 * (2.75 / 12.0) * M_PI
              </code>
            </pre>

          </div>
          <div>
            <p>INSERT IMAGES</p>
          </div>
          <div>
            <h3 className="mt-[2rem] mb-[1rem] ml-[4rem] mr-[4rem] text-[1.2rem] font-bold">Odometry</h3>
            <p className="justify-start mt-[1rem] mb-[2rem] ml-[4rem] mr-[4rem]">Two-wheel odometry is a method of estimating a robot's position and orientation by using the data from two wheels. By tracking the rotation of each wheel, we can determine how far the robot has traveled and how much it has turned. This information is then used to update the robot's estimated position and orientation in a process called pose estimation.</p>

            <pre className="bg-black/[.05] dark:bg-white/[.06] mx-[4rem] px-[1rem] py-[1rem] rounded font-[family-name:var(--font-geist-mono)] font-semibold text-wrap">
              <code>
                const double REF_TO_WHEEL_DIST = 0.25; // feet {"\n\n"}

                struct data {"{"}{"\n"}
                  {"  "}double x;{"\n"}
                  {"  "}double y;{"\n"}
                  {"  "}double theta;{"\n"}
                {"}"};{"\n\n"}

                struct data current_pose, prev_pose;{"\n\n"}

                void update_pose (double delta_left, double delta_right) {"{"}{"\n"}
                  {"  "}double delta_dist = (delta_right + delta_left) / 2;{"\n"}
                  {"  "}double delta_theta = (delta_right - delta_left) / (2 * REF_TO_WHEEL_DIST);{"\n\n"}

                  {"  "}current_pose.x += delta_dist * cos(current_pose.theta + delta_theta / 2);{"\n"}
                  {"  "}current_pose.y += delta_dist * sin(current_pose.theta + delta_theta / 2);{"\n"}
                  {"  "}current_pose.theta += delta_theta;{"\n\n"}

                  {"  "}while (current_pose.theta {">"} 2 * M_PI) current_pose.theta -= 2 * M_PI;{"\n"}
                  {"  "}while (current_pose.theta {"<"} 0) current_pose.theta += 2 * M_PI;{"\n"}
                {"}"}
              </code>
            </pre>

            <p className="justify-start mt-[1rem] mb-[2rem] ml-[4rem] mr-[4rem]"><code className="bg-black/[.05] dark:bg-white/[.06] px-[6px] py-[3px] rounded font-semibold">delta_left</code> and <code className="bg-black/[.05] dark:bg-white/[.06] px-[6px] py-[3px] rounded font-semibold">delta_right</code> are given to the function as the current encoder count minus the previous encoder count.</p>
          </div>
          <div>
            <p>INSERT IMAGES</p>
          </div>
          <div>
            <h3 className="mt-[2rem] mb-[1rem] ml-[4rem] mr-[4rem] text-[1.2rem] font-bold">Wandering Function</h3>
            <p className="justify-start mt-[1rem] mb-[2rem] ml-[4rem] mr-[4rem]">The wandering function utilizes the ultrasonic sensor on the robot, which is mounted on a servo motor, to wander around a room. The robot will always try to drive forwards, but will continuously check if an obstacle is detected 20cm in front of it. If an obstacle is detected, the robot will immediately stop driving forwards, and go through a check of the left and right sides of the robot to identify which direction is free for it to move to. It triggers the servo to look left, then capture the distance to any objects to the left, then looks right to again capture the distance to any objects to the left. If nothing is detected in one direction, but something is detected in the other, it will trigger the robot to rotate 90 degrees in the free direction and continue driving forward. The robot defaults to rotating and driving left if both sides are free. This whole process will continue to run until the user stops the robot by changing modes back to USER mode.</p>
          </div>
          <div>
            <p>INSERT IMAGES</p>
          </div>
          <div>
            <h3 className="mt-[2rem] mb-[1rem] ml-[4rem] mr-[4rem] text-[1.2rem] font-bold">Homing Function</h3>
            <p className="justify-start mt-[1rem] mb-[2rem] ml-[4rem] mr-[4rem]">The homing function works by utilizing the position and rotation values calculated by the robot as it operates and drives around. When the homing function is triggered by the user, the robot first stops, then utilizes its current position to calcuate a desired heading to aim at the origin (0, 0) where it started driving from. This calculation looks like</p>

            <pre className="bg-black/[.05] dark:bg-white/[.06] mx-[4rem] px-[1rem] py-[1rem] rounded font-[family-name:var(--font-geist-mono)] font-semibold text-wrap">
              <code>
              double normalize_angle_0_to_2pi(double angle) {"{"}{"\n"}
              {"  "}angle = fmod(angle, 2 * M_PI);{"\n"}
                {"  "}if (angle {"<"} 0){"\n"}
                {"    "}angle += 2 * M_PI;{"\n"}
                  {"  "}return angle;{"\n"}
                {"}"}{"\n\n"}

              double calculate_target_angle(double x_r, double y_r, double theta_r, double x_t, double y_t) {"{"}{"\n"}
              {"  "}double dx = x_t - x_r;{"\n"}
                {"  "}double dy = y_t - y_r;{"\n\n"}

                {"  "}double theta_target = atan2(dy, dx);{"\n"}
                {"  "}double theta_relative = theta_target - theta_r;{"\n\n"}

                {"  "}return normalize_angle_0_to_2pi(theta_relative);{"\n"}
                {"}"}
              </code>
            </pre>

            <p className="justify-start mt-[1rem] mb-[1rem] ml-[4rem] mr-[4rem]">After this calculation, we utilize a function to rotate the robot in place to this angle within a certain tolerance.</p>

            <pre className="bg-black/[.05] dark:bg-white/[.06] mx-[4rem] px-[1rem] py-[1rem] rounded font-[family-name:var(--font-geist-mono)] font-semibold text-wrap">
              <code>
              case s_HOMING: {"{"}{"\n"}
              {"  "}if (isNewState) {"{"}{"\n"}
              {"    "}left_start = left_counts;{"\n"}
              {"    "}right_start = right_counts;{"\n"}
              {"    "}target_theta_deg = radToDeg(calculate_target_angle(current_pose.x, current_pose.y, current_pose.theta, 0, 0));{"\n"}
              {"    "}turning = true;{"\n"}
              {"    "}hasHomedAngle = false;{"\n"}
              {"    "}Motor_Right(14999, 13999);{"\n"}
              {"  "}{"}"}{"\n\n"}
              {"  "}if (hasCompletedTurn(left_counts - left_start, right_counts - right_start, target_theta_deg * (180.0 / 90.0))) {"{"}{"\n"}
              {"    "}Motor_Stop();{"\n"}
              {"    "}Clock_Delay1ms(1000);{"\n"}
              {"    "}turning = false;{"\n"}
              {"    "}hasHomedAngle = true;{"\n"}
              {"    "}state = s_FORWARD;{"\n"}
              {"  "}{"}"}{"\n"}
              {"  "}break;{"\n"}
              {"}"}
              </code>
            </pre>

            <p className="justify-start mt-[1rem] mb-[1rem] ml-[4rem] mr-[4rem]">Finally, the robot drives forward until it reaches within a 1.5 foot radius around the origin, at which point the robot stops. This high tolerance was necessary since the accuracy of the robot's driving and tracking is not very good.</p>

          </div>
          <div>
            <p>INSERT IMAGES</p>
          </div>
          <div>
            <h3 className="mt-[2rem] mb-[1rem] ml-[4rem] mr-[4rem] text-[1.2rem] font-bold">Bluetooth, Controls, and FSMs</h3>
            <p className="justify-start mt-[1rem] mb-[2rem] ml-[4rem] mr-[4rem]">The robot's operation is controlled using a mixture of hardware interrupts, which updates the encoder counts, and Finite State Machines (FSMs). This robot features two FSMs, one that controls the mode it is in, and another that controls the state of the motors. There are 3 modes, <code className="mt-[1rem] mb-[1rem] bg-black/[.05] dark:bg-white/[.06] px-[6px] py-[3px] rounded font-semibold">m_USER</code> mode, <code className="mt-[1rem] mb-[1rem] bg-black/[.05] dark:bg-white/[.06] px-[6px] py-[3px] rounded font-semibold">m_HOMING</code> mode, and <code className="mt-[1rem] mb-[1rem] bg-black/[.05] dark:bg-white/[.06] px-[6px] py-[3px] rounded font-semibold">m_WANDER</code> mode. There are 6 motor states, <code className="mt-[1rem] mb-[1rem] bg-black/[.05] dark:bg-white/[.06] px-[6px] py-[3px] rounded font-semibold">s_STOP</code>, <code className="mt-[1rem] mb-[1rem] bg-black/[.05] dark:bg-white/[.06] px-[6px] py-[3px] rounded font-semibold">s_FORWARD</code>, <code className="mt-[1rem] mb-[1rem] bg-black/[.05] dark:bg-white/[.06] px-[6px] py-[3px] rounded font-semibold">s_BACKWARD</code>, <code className="mt-[1rem] mb-[1rem] bg-black/[.05] dark:bg-white/[.06] px-[6px] py-[3px] rounded font-semibold">s_ROT_L_90</code> (rotates 90 degrees to the left), <code className="mt-[1rem] mb-[1rem] bg-black/[.05] dark:bg-white/[.06] px-[6px] py-[3px] rounded font-semibold">s_ROT_R_90</code> (rotates 90 degrees to the right), and <code className="mt-[1rem] mb-[1rem] bg-black/[.05] dark:bg-white/[.06] px-[6px] py-[3px] rounded font-semibold">HOMING</code> (points robot at origin).</p>

            <p className="justify-start mt-[1rem] mb-[2rem] ml-[4rem] mr-[4rem]">This robot is controllable using Bluetooth from a mobile smartphone. The user is able to change the modes and states the robot is in by sending an integer to the robot. The different commands sendable to the robot are shown below.</p>

            <pre className="bg-black/[.05] dark:bg-white/[.06] mx-[4rem] px-[1rem] py-[1rem] rounded font-[family-name:var(--font-geist-mono)] font-semibold text-wrap">
              <code>
              switch (command) {"{"}{"\n"}
	              {"  "}case 0: {"{"}state = s_STOP; break;{"}"}{"\n"}
	              {"  "}case 1: {"{"}state = s_FORWARD; break;{"}"}{"\n"}
	              {"  "}case 2: {"{"}state = s_BACKWARD; break;{"}"}{"\n"}
	              {"  "}case 3: {"{"}state = s_ROT_L_90; break;{"}"}{"\n"}
	              {"  "}case 4: {"{"}state = s_ROT_R_90; break;{"}"}{"\n"}
	              {"  "}case 5: {"{"}state = s_ROT_180; break;{"}"}{"\n"}
	              {"  "}case 6: {"{"}mode = m_HOMING; break;{"}"}{"\n"}
	              {"  "}case 7: {"{"}mode = m_WANDER; break;{"}"}{"\n"}
	              {"  "}case 8: {"{"}state = s_STOP; current_pose.x = 0; current_pose.y = 0; current_pose.theta = 0; left_counts = 0; right_counts = 0; break;{"}"}{"\n"}
	              {"  "}default: {"{"}state = s_STOP; break;{"}"}{"\n"}
              {"}"}
              </code>
            </pre>
          </div>
        </div>
        
        {/* <pre className="bg-black/[.05] dark:bg-white/[.06] px-[2rem] py-[2rem] rounded font-[family-name:var(--font-geist-mono)] font-semibold">
          <code>
            void update_pose(double delta_left, double delta_right) &#123;{"\n"}
              double delta_dist = (delta_left + delta_right) / 2;{"\n"}
              double delta_theta = (delta_right - delta_left) / (2 * ref_to_wheel_dist);{"\n"}
      {"\n"}
              current_pose.x += delta_dist * cos(current_pose.theta + delta_theta / 2);{"\n"}
              current_pose.y += delta_dist * sin(current_pose.theta + delta_theta / 2);{"\n"}
              current_pose.theta += delta_theta;{"\n"}
            &#125;
          </code>
        </pre> */}
      </main>
    </div>
  );
}
