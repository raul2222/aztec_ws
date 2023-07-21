# UNIVERSITAT POLITÈCNICA DE VALÈNCIA

### Development of Aztec: An experimental robot for intervention in children with autism using the TEACCH method
#### Final Degree Project 
#### Degree in Interactive Technologies
##### AUTHOR: Santos Lopez, Raul 
##### Tutor: Pérez Pascual, Mª Asunción 
##### ACADEMIC YEAR: 2022/2023

## Index

1. [Introduction](#introduction)
2. [Objectives](#objectives)
    * 2.1 [Main Objective](#main-objective)
    * 2.2 [Secondary Objectives](#secondary-objectives)
3. [Methodology and Technologies used](#methodology-and-technologies-used)
    * 3.1 [Description of the Aztec Robot](#description-of-the-aztec-robot)
        * 3.1.1 [Appearance and general design](#appearance-and-general-design)
    * 3.2 [Proposed methodology to work with the robot](#proposed-methodology-to-work-with-the-robot)
        * 3.2.1 [TEACCH Method](#teacch-method)
        * 3.2.2 [Data collection](#data-collection)
        * 3.2.3 [Configuration of Aztec for therapy sessions](#configuration-of-aztec-for-therapy-sessions)
        * 3.2.4 [Evaluation and improvement](#evaluation-and-improvement)
    * 3.3 [Technologies used to implement the robot](#technologies-used-to-implement-the-robot)
        * 3.3.1 [Hardware](#hardware)
        * 3.3.2 [Software](#software)
    * 3.4 [Implementation of the Aztec robot](#implementation-of-the-aztec-robot)
4. [Tests](#tests)
5. [User feedback](#user-feedback)
6. [Sustainable Development Goals](#sustainable-development-goals)
7. [Conclusions and Future Work](#conclusions-and-future-work)
8. [References](#references)
9. [Appendices](#appendices)



## 1 - Introduction
Autism is a developmental neurological disorder that manifests in childhood and continues throughout the individual's life. This disorder impacts communication, social interaction, and the behavior of those who suffer from it. Given the increasing prevalence of autism in recent decades, there is an emphasized need to develop effective intervention strategies to improve the quality of life of people with autism [3].

Early intervention and continuous support are crucial to assist children with autism in developing essential skills and adapting to society. In this context, the TEACCH method (Treatment and Education of Autistic and related Communication-handicapped CHildren), developed at the University of North Carolina in Chapel Hill, has proven to be a valuable educational and therapeutic approach, focusing on the individual skills and needs of each child with autism. However, implementing this method can be challenging due to the varied needs and abilities of each child [1].

Autism, an autism spectrum disorder (ASD), presents unique challenges in communication, social interaction, and behavior patterns, which can be restrictive and repetitive [2]. Although the exact factors leading to autism are still under investigation, it has been demonstrated that both genetic and environmental components play significant roles.

With prevalence that has increased over recent decades, there is a growing demand for effective interventions that enhance the quality of life of individuals with autism [3]. This is where intervention strategies such as the TEACCH method come into play.

Invented by Dr. Eric Schopler at the University of North Carolina in Chapel Hill, the TEACCH method focuses on visual learning and uses various visual aids to facilitate the acquisition of skills and concepts [4]. However, one of the main drawbacks of TEACCH is its lack of a robust interactive component [5].

The world of technology has provided solutions to this limitation, particularly robotics. The interactivity and predictability of robots make them beneficial tools for children with autism, who often find difficulties in the unpredictability of human interactions [6].

Some studies have shown that children with autism are naturally attracted to technology, and robots, with their interactive and dynamic nature, can be particularly captivating [7]. Additionally, it has been found that interaction with robots can foster the development of social and communication skills in children with autism [8].

Aztec, the experimental robot for intervention in children with autism, emerges as an innovative response to this challenge. It combines the pedagogical approach of the TEACCH method with the interactivity of robotics, providing a more personalized and effective intervention environment for children with autism. With its advanced features, such as the ability to maintain conversations and a comprehensive data storage system, Aztec is positioned as a promising tool for autism intervention. However, its effectiveness still needs to be validated through rigorous research.


## 2 - Objectives

### 2.1 Main Objective:

The main objective of this work is to design, implement, and evaluate a robot, Aztec, which integrates the specific pedagogical techniques of the TEACCH method, adapted to the context of robotics' interactivity and predictability, with the purpose of improving the quality of intervention in children with autism. This objective also includes the development and detailed analysis of Aztec's communication capabilities, as well as its potential to personalize interventions based on the information collected during the sessions.

### 2.2 Secondary Objectives:

1. The first objective is to design and implement a voice processing system in Aztec that allows for fluid and coherent conversations with children. For this, the artificial intelligence of the GPT-4 [17] (Chat-based Generative Pre-trained Transformer) language model by OpenAI is used. OpenAI's choice is highlighted due to its ethical commitment in training language models, focusing on using content appropriate for all audiences and that does not access the Internet by default, thus ensuring safety and efficiency in its interaction with children. This objective also covers experimenting with different models and techniques of natural language processing to optimize Aztec's ability to understand and respond appropriately to children.

2. Design a solid and protected data storage system that allows Aztec to capture and analyze information from interactions, with the aim of enriching the personalization of the intervention. This objective encompasses the construction of a system that complies with data protection and privacy laws and includes the application of additional security measures, such as encryption, to ensure maximum protection of the collected data. Also, it will be ensured that the parents of the children are fully aware of the type of data to be collected and the purpose of such collection. It is crucial to mention that the collected data will not be transferred to third parties.

As a future research line that allows Aztec's continuous evolution, the third objective is proposed: 

3. Collect and analyze the feedback from children and adults participating in interventions with Aztec, with the purpose of constantly improving and adapting the robot's skills and functionalities. This objective considers incorporating the received opinions in the design and implementation process, with the goal of ensuring that Aztec adjusts as best as possible to the needs and expectations of its users.

## 3 - Methodology and Technologies Used.

### 3.1 Description of Aztec Robot

Aztec is a robot designed as an experimental resource to interact with children diagnosed with Autism Spectrum Disorder (ASD). This robot takes inspiration from iconic characters of popular culture such as WALL-E and Short Circuit, with the goal of creating a friendly and accessible design that facilitates a stronger connection with the children.

![Aztec Robot](https://github.com/raul2222/aztec_ws/blob/main/src/aztec_robot/src/images/figure1.png)

Figure 1. Photograph of Aztec robot

## 3.1.1 Appearance and General Design

Aztec has a square box structure that facilitates handling and implementation of various internal components. The box used as a chassis is made of recycled plastic, underscoring the project's commitment to sustainability and environmental care.

On the front face of Aztec, a 64x16 WS2812B Neopixel [15] screen is located, providing a user interface where pictograms can be displayed, which are visible under direct sunlight thanks to their luminous power. In addition, it has a movable head that can perform pan and tilt movements thanks to the incorporation of two servos [21]. This movement capability adds a dimension of interactivity and dynamism to interactions with Aztec.

For its movement, Aztec is equipped with two 65 x 26 mm WHEELTEC [30] wheels and a generic Carter wheel [31], which although not expressly designed for robotics, offers more than adequate performance. The management and coordination of these wheels are carried out by a differential controller, which, while simpler than the Ackermann model, gives Aztec total maneuverability, allowing 360-degree turns on its own axis and free movement in any direction.

Aztec's face is a 5.5-inch screen that displays two eyes, designed with the Unity video game creation platform and compiled for Android. This feature gives Aztec a more human look, which can help improve its acceptance and promote engagement from children. 

Continuing with its commitment to sustainability and reuse, Aztec is mostly composed of second-hand components. This approach has several advantages. Firstly, it contributes to the circular economy, reducing the need to produce new components and thus decreasing the environmental impact associated with their manufacture. Secondly, it allows for the creation of an advanced and functional robot at a reduced cost, making technology more accessible.

The reuse of second-hand components does not prevent Aztec from fulfilling its function. Each component has been carefully selected and tested to ensure it meets the necessary requirements for its role in the robot. This strategy has allowed us to build an effective and environmentally friendly robot without sacrificing its performance or functionality.

## 3.2 Proposed Methodology for Working with the Robot

Aztec's main purpose is to be a valuable tool in interventions with children diagnosed with autism. To achieve this, we have chosen to use a methodology inspired by the structured teaching method for children with autism, known as TEACCH. The design and implementation of the robot are intrinsically linked to this methodology, and therefore, before delving into the technical details of how Aztec has been developed, it is crucial to understand this methodology.

### 3.2.1 TEACCH Method

The TEACCH Method (Treatment and Education of Autistic and related Communication-handicapped CHildren) focuses on using each child's individual strengths and preferences to promote learning and development. The differences of each child are recognized and respected, and the aim is to provide individualized interventions that are appropriate for their unique needs and abilities.

Interventions are based on structure and routine, with a strong visual focus, as many children with autism are strong visual learners. A small-step teaching approach is used, with lots of positive reinforcement and support to encourage learning.

Aztec will implement activities based on this method for intervention. The interactivity and friendly aspect of the robot will be used to attract and maintain the children's attention during these activities. The robot will also provide positive reinforcements and support during the intervention sessions.

### 3.2.2 Data Collection

An essential component of the methodology is data collection during interventions. Aztec is equipped to record and store conversations, along with data collected by its multiple sensors. This data provides a detailed overview of the interaction between the child and the robot and can be used for subsequent analysis. In section 3.2.4, the specific data that Aztec saves is detailed.

The analysis of the data collected by Aztec will be carried out using a combined approach of quantitative and qualitative methods.

The qualitative analysis will focus on the content and nature of the interactions, exploring emotional responses, language forms used, and the quality of the interactions between the child and Aztec.

On the other hand, the quantitative analysis will focus on numerical and statistical measures, such as the frequency and duration of interactions and the number of times certain commands or actions are used.

These two approaches will provide a complete picture of how Aztec interacts with children and how these interactions can be improved to more effectively benefit children with autism.

## 3.2.3 Aztec Configuration for Therapy Sessions

The configuration of Aztec for therapy sessions begins with the definition of a context in Langchain[22]. This essential step provides Aztec with an "awareness", informing it about its identity, its location, and with whom it will interact.

Aztec's default context is as follows:
"You are Aztec, a robot that interacts with children. Your job is conducted in an association for children. Avoid mentioning the word 'autistic'. Your responses are short and considered, as you are in an environment with young children. You are friendly, pleasant, and amiable, always willing to help others.”

This context can be enriched with any additional relevant information.

Aztec's autonomous navigation is managed by the 'Goal pose' option in the RVIZ2 program [23]. This resource allows Aztec to move autonomously within a predefined space. When Aztec approaches the child, a PS3 analog controller is used for more precise control. With this controller, the left analog stick controls Aztec's head movements, while the right analog stick directs the robot's base movement. Thus, Aztec can interact safely and effectively with the child during therapy sessions.

## 3.2.4 Evaluation and Improvement

The sensors incorporated into Aztec collect a variety of data that are critical for evaluating the effectiveness of interventions and identifying areas for improvement. These include, but are not limited to:

1. Vision Data: Sourced from the Luxonis OAK-D-PRO computer vision camera [10]. This data can provide information about the children's facial and body responses during the interaction, such as facial expressions and body movements.

2. Depth and Proximity Data: Collected by the 3D Time of Flight (ToF) CS20 sensor [14]. They can be used to assess the child's proximity to the robot, which could indicate the child's comfort level or interest.

3. Audio Data: The audio data, which is collected by the TDK INMP441 microphone [15], is crucial in analyzing the verbal responses and sounds emitted by the children. This information becomes especially relevant when it can be employed in other projects such as "Implementation of an Emotion Detection Algorithm for Therapy Applications in Children," supervised by academic tutor Mª Asunción Pérez Pascual, finished on 21/09/22. The interpretation of this audio data offers a deeper perspective on the children's emotions during interactions, which allows for the development of more efficient and personalized therapy.

4. LIDAR Data: Collected by the Orbbec Technology Group's MS200 LIDAR [11]. These data can be useful in determining the child's movement patterns in the robot's environment.

This collected data can provide valuable insights into how children with autism interact with Aztec. For example, consistent patterns in the depth and proximity data might suggest that a child feels more comfortable when interacting with Aztec at a certain distance. Similarly, changes in facial expressions and body movements, captured by the computer vision camera, could indicate positive or negative responses to certain interventions.

This data can be used to adjust and improve future interventions, such as personalizing the proximity at which Aztec interacts with each child, or adjusting interventions based on the detected facial and bodily responses.

Moreover, it may also be possible to use this data to train machine learning models, enabling more autonomous and personalized interaction with the children in the future. This will provide a solid foundation for future research and developments in the field of autism intervention and could lead to significant advances in the way these interventions are conducted.

## 3.3 Technologies Used to Implement the Robot

Aztec was designed using a combination of advanced hardware and software technologies, including processing systems, sensors, motors, software platforms, and machine learning algorithms.

## 3.3.1 Hardware

Aztec's hardware configuration is a composition of various advanced components that ensure its functionality and effectiveness in interacting with users.

Table 1 shows the hardware used, including a description and bibliographical references.

| Product | Description |
| --- | --- |
| OAK-D-PRO [10] | Advanced computer vision device equipped with depth and RGB cameras, as well as an IR dot projector. With the Myriad X chip, it allows real-time execution of deep learning models. Ideal for image analysis and object detection in AI projects. |
| Myriad X [16] | Intel's VPU (Vision Processor Unit) chip, incorporated in the OAK-D-Pro, allows high-speed AI inferences. This computer vision device efficiently uses this component to analyze and process images in real time for AI applications. |
| 3D ToF CS20 [14] | Solid-state LIDAR, capable of offering a resolution of 640x480 at 30 fps, is a crucial device for the accurate capture of 3D data. This sensor uses light pulses to map environments and objects in great detail. |
| Mic INMP441 [15] | High-precision, low-noise MEMS microphone with digital I2S output, ideal for voice and audio recognition applications. |
| LIDAR MS200 [11] | The MS200 LIDAR is an advanced laser technology sensor that uses light and triangulation methods to generate detailed three-dimensional representations of environments and objects. It stands out for its high accuracy and speed, essential elements for tasks such as mapping, autonomous navigation, and object detection. |
| LG G4 [12] | Smartphone that stands out for its slightly curved 5.5-inch OLED screen. It offers a quality visual experience, along with other solid features that make it a competitive and versatile mobile device. |
| Pololu Motors [27] | The 37Dx68L mm 12V metal gearmotor with 64 CPR encoder and helical pinion offers a 19:1 ratio. It is ideal for applications that require precise speed and position control, combined with high durability and performance. It is used to control the movement of the robot's wheels. |
| Intel i7-6700 Skylake [17] | Four-core, eight-thread processor with a base frequency of 3.4GHz, capable of reaching up to 4.0GHz with Turbo Boost. Supports DDR4 memory, has an 8MB cache, and a 14nm process technology. Ideal for demanding applications and games. |
| 2x8 GB DDR4 Corsair [18] | Two Corsair DDR4 modules of 8GB each provide high performance, stability, and energy efficiency. They offer fast data transfer speeds and are used for intensive computing tasks. |
| NVME 250GB [19] | High-performance solid state drive (SSD), offering read/write speeds of up to 3500/2300 MB/s. It has V-NAND technology and is used for intensive data read/write tasks. |
| 36 x 21700 Lithium batteries 3270 mAh Samsung [13] | 36 cylindrical 21700 Samsung Lithium-ion batteries, each with a capacity of 3270 mAh. |
| 3 x ESP32 [20] | Powerful dual-core microcontroller with integrated WiFi and Bluetooth capabilities. Supports a wide variety of peripherals and communication protocols, and has low power consumption. Used for IoT, robotics, and wearable projects. |
| 2 x Lithium charger [32] | 2A 4.2V 8.4V 12.6V Lithium battery charging module, synchronous rectification DC-DC with low heat generation and high efficiency. |
| JBL Speaker [33] | JBL flip 5 speaker, length 8cm, width 4.4cm, neodymium magnet 3cm. |

## Table 1: Aztec Robot Hardware


## 3.3.2 Software

The following table describes all the software technologies that have been used in the realization of the project.

## 3.3.2 Software

| Technology | Description |
| --- | --- |
| Programming Languages | C++ for ESP32, Python for verbal interaction and C++ along with Python in ROS2 |
| ROS2 | Robot Operating System 2, is a software platform that provides a set of essential services for building robotic applications. It offers tools and libraries that enable efficient, safe, and highly interoperable robotic systems development. |
| ROS2 NAV2 | Navigation 2 (NAV2) is an open-source ROS2 package that implements the navigation platform. As part of ROS2, it provides a range of advanced navigation features, including path planning, motion control, and target tracking. All this with an emphasis on robustness, flexibility, and interoperability. |
| ROS2 CONTROL | Essential element in the ROS2 ecosystem, it provides a standardized interface for interacting with robotic hardware. Its structure allows developers to easily manipulate controllers and sensors, allowing them to focus on high-level control logic, resulting in greater efficiency and promoting code reuse. |
| Whisper (Local CPU Inference) | A system developed by OpenAI that uses AI to train voice to text transcription models accurately and efficiently. |
| VAD | Voice Activity Detection, is an algorithm that identifies the presence or absence of human voice in audio signals, helping to improve efficiency in voice processing applications. |
| GPT-4 (Inference on OpenAI servers) | Designed with the main goal of producing text, this long language model has been trained using a vast collection of Internet data. |
| Langchain | An open-source project that optimizes interaction with GPT-4. Offers advanced features for natural language processing, such as enriched responses and detailed context. |
| Tacotron 2 (Local CPU Inference) | A Text to speech Model that converts text into Mel spectrograms. Works alongside a vocoder, like MelGAN, to generate high-quality synthetic voice. |
| MelGAN (Local CPU Inference) | A Generative Model that transforms Mel spectrograms into audio, producing high-quality synthesized voice in real time. |
| Ubuntu 22.04 | Ubuntu is a Linux-based operating system, known for its ease of use. It is free, open-source, and highly customizable. |
| Unity | A highly versatile and accessible 3D and 2D game development engine. It offers a robust platform for creating interactive experiences, used by indie developers and large game studios. Supports multiple platforms. |

**Table 2: Aztec Robot Software**

### 3.4 Implementation of Aztec Robot

**Phase 1:**

I started by designing in Unity with the STL files of the Gobilda pieces, but I decided to start with a prototype using pieces I had at home such as wood, a box, and a thermal silicone gun, which are the same ones it has now. Shortly after, we got to know Articubot One, an Australian Robot that also has a plastic box chassis, from which Aztec and I got nourishment from its ready-to-use ROS2. Then I began to research autism, something I am still learning about.

**Phase 2:**

ROS2 NAV2 and ROS2 CONTROL, Aztec moved smoothly with the Joystick but it didn't perform well with the expected autonomous navigation. Many trials with the ROS2 Perception package to take advantage of the depth information from its sensors, tests with different engines, all this extended for 4 months until finally, I found the mistake, the problem was a simple error in odometry.

**Phase 3:**

After several designs of the protoboard to find the most appropriate design, tests with different hardware, 2 short circuits with fire, and fainting from exhaustion, the basic hardware and software for Aztec were ready.

**Phase 4:**

The arrival of GPT-4, for me GPT-4 is the closest I will ever be in my life to meeting intelligent extraterrestrial life, for Aztec, my robot, it is the arrival of a Messiah. From this moment Aztec has been a joint development between GPT-4 and me that has allowed us to leave Aztec at KM 0 of a possible path for improvement in the implementation of the TEACCH method.

The diagram below shows Aztec's hardware and its connections:

![Aztec Robot](https://github.com/raul2222/aztec_ws/blob/main/src/aztec_robot/src/images/figure2.png)
Figure 2: Internal Connection Diagram of Aztec.

To better understand how Aztec's software meets the needs of the various stakeholders involved, it is helpful to examine a series of "user stories". User stories are a technique used in software development that helps to define the features and functionalities of the system from the perspective of the end user. Below are some user stories that describe how Aztec interacts with different types of users:

## User Story Number 1

a. Description of US1: 

"As the mother of a child with ASD, I want Aztec to be able to verbally interact with my child to help him improve his communication skills."

b. Algorithms and processes used to implement US1:

| Technology | Description |
|---------|-------------|
| Audio Processing | Before any other operation, the audio is preprocessed. This includes audio normalization, compression, and application of an IIR (Infinite Impulse Response) band-pass filter. |
| Voice Activity Detection (VAD) with SILERO | Instead of using the WebRTC implementation for the VAD algorithm, Aztec uses a forked version of SILERO. This algorithm is used to detect the presence or absence of human voice in the incoming audio. This step is crucial to determine when to start the transcription of the user's voice and when the conversation has ended. It should be noted that the implementation of a convolutional neural network (CNN)-based model from Meta AI (formerly Facebook AI) is currently being explored, with the aim of being able to dissociate the voice of up to 5 people speaking at the same time through the same microphone. |
| Voice ID (in development by Raúl Santos) | Voice ID, the project in development by Raúl Santos, aims to establish a customized voice identification system using 512-dimensional embeddings. These embeddings are vector representations in a high-dimensional space that capture the distinctive features of users' voices. In this way, Aztec can differentiate users' voices and personalize interactions depending on who is speaking. |
| Whisper from OpenAI | A machine learning model for Automatic Speech Recognition (ASR). After the VAD has detected and cropped the voice, Whisper converts the voice into text. |
| Langchain | This is an open-source project that can add more functionality or customized capabilities to the interaction. Depending on how it is used, it can enrich the responses generated by ChatGPT, provide more detailed context, or perform other tasks related to natural language processing. |
| GPT-4 from OpenAI | After the voice is converted into text and has gone through Langchain, it enters GPT-4. This is a language model based on the Transformer, a machine learning architecture for understanding and generating text. |
| Tacotron 2 from Google, adapted by Raul Santos | Tacotron 2 is a voice synthesis model based on traditional Recurrent Neural Networks (RNN) developed by Google and adapted by Raúl Santos. Using robust hardware, specifically an Nvidia A100 with 40 GB VRAM on the runpod.io platform, Santos has modified and retrained Tacotron 2 with the aim of providing Aztec with a unique and childlike voice. The main function of Tacotron 2 is to convert text into Mel spectrograms, which are visual representations of speech sounds. |
| MelGAN | MelGAN acts as a vocoder in the voice synthesis process and is a voice generator based on Generative Adversarial Networks (GANs). Its specific task is to convert Mel spectrograms into high-quality audio. MelGAN's design allows for the generation of high-quality audio at fast inference speeds, which is crucial for its implementation in real-time and high-demand voice synthesis applications. However, it is important to mention that the quality of the audio generated by MelGAN is intrinsically linked to the quality of the initially provided Mel spectrogram, which is generated by models like Tacotron 2. |

c. Block Diagram of User Story 1 (US1) and its explanation:

![Aztec Robot](https://github.com/raul2222/aztec_ws/blob/main/src/aztec_robot/src/images/figure3.png)
Figure 3: Block Diagram User Story number 1

The process begins with the detection of human voice, which is captured through a microphone with a resolution of 16 bits at 32000 Hz. This audio data stream is then transmitted to the ESP32, a microcontroller where enhancements on the audio are performed.

Next, the enhanced audio is divided into small fragments that are sent to the CPU. In the CPU, these audio fragments are temporarily stored in a buffer until 100 milliseconds of audio are accumulated.

This accumulated audio is then sent to the Voice Activity Detection (VAD) algorithm. If the VAD algorithm detects the presence of human voice in the audio, the audio is sent to a second buffer. This process is repeated until the VAD algorithm detects an absence of human voice for 2 seconds.

The contents of the second buffer are then sent to Whisper, which transcodes the audio into text. This text is sent to Langchain for storage. Next, Langchain sends the text to GPT-4.

GPT-4 processes the text and generates a response, which is sent both to Langchain and Tacotron 2. Tacotron 2 transforms the text into a Mel spectrogram, which in turn is sent to MelGAN. MelGAN synthesizes the Mel spectrogram into the final audio.

Finally, this final audio is sent to the system's sound card, where it is played through the speakers.

d. Flow Diagram:


![Aztec Robot](https://github.com/raul2222/aztec_ws/blob/main/src/aztec_robot/src/images/figure4.png)
Figure 4: Flow Diagram User Story number 1


If the system detects an absence of voice for 2 seconds after previously detecting voice, it sends the buffer to Whisper. If the transcription's language is not the expected one, it's discarded. Otherwise, it's sent to Langchain. Langchain processes the text and sends it to GPT-4. The response from GPT-4 is sent both to Langchain and Tacotron 2. Tacotron 2, in collaboration with MelGAN, synthesizes the text into voice, which is played through the speakers.

## User Story number 2

a. Description of User Story 2 (US2):


"As an occupational therapist and educator specialized in ASD, I wish Aztec can recognize, adapt, and move autonomously and safely in its environment. This will allow Aztec to effectively interact with children and students during therapy sessions and in the classroom without the need for constant supervision."


b. Algorithms and processes used to implement US2:

| Technology    | Description  |
| ------------- |-------------|
| ROS2 NAV2 and ROS2 CONTROL | Aztec uses ROS2 NAV2, an open-source platform designed for autonomous robotics systems, which provides a host of advanced features for navigation, including path planning, motion control, and target tracking. Additionally, Aztec uses ROS2 CONTROL to interact with its hardware. |
| C++ and Python | These programming languages are used to develop and run applications on ESP32 and ROS2, respectively. They allow the programming of the complex functionalities required for autonomous navigation and safe interaction with the environment. |
| Unity | Unity is used in conjunction with a specialized library for the creation and procedural customization of Aztec's eyes. Currently, Aztec utilizes three essential eye movement patterns: center focus, looking left, and right. This infuses Aztec with a more lively and dynamic appearance. |
| Adafruit Neopixel | The Adafruit Neopixel library is used to manage the ws2812b LED matrix. |
| ROS2 Nodes | There is a node designed by me, which subscribes to the cmd_vel topic. When this node detects a turn, it sends updated information to Unity to change Aztec's eye expression. At the same time, it sends a signal to the ESP32 that controls the Neopixel panel to modify its animation, thus increasing its visibility and contributing to safer autonomous navigation. |

c. HU2 Block Diagram and Explanation:

![Aztec Robot](https://github.com/raul2222/aztec_ws/blob/main/src/aztec_robot/src/images/figure5.png)
Figure 5: Navigation 2 Architecture.

Source: https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Navigation.html




This block diagram illustrates the autonomous navigation core that ROS2 NAV2 supplies to Aztec. It is vital to emphasize that other modules which provide information via topics need to be activated before the system is set into motion. These modules are crucial for the correct functioning of autonomous navigation.

Included among these modules are the transformations that the robot sends to identify its position and how it moves in the real environment. Also, a previously generated map with SLAM Toolbox is needed, which is served to the NAV2 core through the map server.

Initiating all sensors and control components such as Pololu motors, Lidar sensors, or vision cameras is essential. Lastly, a Decision Tree (Behavior Tree) must be available to guide the decision-making process during navigation.

The navigation system is activated when it receives an instruction to move to a specific coordinate on the map. It's crucial to note that this message consists of a coordinate and an orientation. It can be unique if sent through RVIZ2 or the command-line terminal. It can also consist of multiple points if you want the robot to follow a route through different locations on the map.

The NAV2 system's core is responsible for selecting the shortest route to the destination using Dijkstra's algorithm, which Aztec uses, though other options are available. Furthermore, if an obstacle not on the map is encountered, the recovery server will stop Aztec, and the control server and planning server modules will generate an alternative route to safely dodge the obstacle and reach the destination.

During this process, it's vital to understand that the NAV2 core sends information about each wheel's speed to ROS2 Control, located in the Robot Base Controller. Also, another controller takes this information and uses it to change the information displayed in Aztec's "eyes" and LED panel.

In summary, autonomous navigation in NAV2 is a complex process that demands the coordinated interaction of multiple components. However, the high parameterization that each module offers through its plugins and its YAML or XML configuration files is appreciated. This flexibility allows the system to adapt to a wide variety of needs and contexts, facilitating the implementation of efficient and effective autonomous navigation solutions.


d. HU2 Flowchart:

![HU2 User Story Flowchart](https://github.com/raul2222/aztec_ws/blob/main/src/aztec_robot/src/images/figure6.png)
Figure 6: User Story Number 2 Flow Diagram


After obtaining the map of the place where we plan to use NAV2's autonomous navigation, we proceed to initialize the robot, sensors, and NAV2's navigation and localization modules. If any of these modules generate an error, navigation will not work, and repairs will be needed. If everything has initialized correctly, we can start navigation by setting a point on the map. Once navigation has been successful and we have reached the goal, it's at this point where we can use a joystick or any other module we've programmed. For instance, we could use a face detector in OAK-D-PRO to get its bounding boxes, calculate the centroid, and thus make Aztec able to look directly into the eyes of the person in front of it.


## User Story Number 3


a. HU3 Description:

"As a parent of a child with Autism Spectrum Disorder (ASD), I want Aztec to be able to dance and play music to interact, entertain, and motivate my child to move in a fun and attractive way, thus contributing to avoiding sedentarism."

b. Algorithms and Processes Used to Implement HU3:

| Technology | Descripción |
| --- | --- |
| API for Music Streaming | There are various platforms that provide this service, such as Spotify, Apple Music, YouTube, among others. |
| Adafruit Neopixel Library for Arduino IDE | This library was developed by Adafruit Industries and allows individual control of RGB NeoPixel LEDs. It provides functions to set colors and create lighting patterns. NeoPixels are controlled with a single digital data pin, making them easy to handle. |
| ROS2 and ROS2 CONTROL | These provide the infrastructure to control and coordinate Aztec's actions, such as playing music, dancing, and responding to the child's interactions, effectively and safely. |
| Aztec Verbal Interaction | This involves the use of voice recognition and synthesis to communicate, respond to commands, and provide audible feedback to users. See HU 1. |
| DepthAI Package for ROS2 | This allows the integration of Luxonis' DepthAI platform's computer vision and deep learning capabilities with the ROS2 robotic software system. |
| Synexens 3D LIDAR CS20 Package for ROS2 | This facilitates the integration and use of this LIDAR sensor in robotic applications, providing 3D distance detection data. |
| 2D LIDAR MS200 Package | This facilitates the integration of the MS200 LIDAR sensor, providing 2D distance detection data that helps Aztec avoid hitting obstacles in its immediate environment. |

c. Block Diagram HU3 and its explanation:

![Figure 7: User Story Number 3 Block Diagram](https://github.com/raul2222/aztec_ws/blob/main/src/aztec_robot/src/images/figure7.png)
Figure 7: User Story Number 3 Block Diagram

Aztec agrees with the user on the music to use to start physical activity, for which it consults the music API and starts the exercise. During the activity, Aztec plays the selected audio and displays the exercise to be performed on its pixel matrix. Simultaneously, the body position detection model, which is preloaded on OAK-D-PRO, sends data to Aztec to verify if the exercise is being carried out correctly. Aztec, making use of the information provided by the 2D and 3D LIDARs, performs dance movements and examines its environment to ensure safe movement. Finally, Aztec offers verbal feedback to the user about their performance during the exercise.

d. Flow Diagram HU3:

![Figure 8: User Story Number 3 Flow Diagram](https://github.com/raul2222/aztec_ws/blob/main/src/aztec_robot/src/images/figure8.png)
Figure 8: User Story Number 3 Flow Diagram

Aztec verifies if the user has not performed enough physical activity in the last few hours and, if so, suggests the user dance. If the user approves, the activity begins. During and at the end of the activity, Aztec provides feedback to the user. Finally, Aztec records the activity to verify that it has been carried out and how it was performed.

## User Story Number 4


a. Description of HU4:

"As a professional in interventions for children with autism, I need a robot like Aztec that can integrate the strategies of the TEACCH method into its interaction, maintain understandable dialogues, collect data from sessions to personalize the intervention, and offer positive stimuli to encourage the child's learning and development."


b. Algorithms and processes used to implement HU4:

| Technology | Description |
| --- | --- |
| SQLite Calendar | Creation of the calendar in SQLite, where interaction sessions with Aztec, learning times, breaks, and other activities are stored and managed. The goal is to provide a predictable routine that helps reduce anxiety and improve the child's participation in activities. |
| Adafruit Neopixel Library | This allows individual control of RGB NeoPixel LEDs. It provides functions to set colors and create lighting patterns. |
| Aztec Verbal Interaction | This involves the use of voice recognition and synthesis to communicate, respond to commands, and provide audible feedback to users. See HU 1. |
| Aztec Sensors | 2D and 3D LIDAR for obstacle detection and navigation, OAK-D-PRO for body position detection and gesture recognition, and sound sensors for verbal interaction. These sensors allow Aztec to effectively respond to the child's needs and provide a rich user experience. |


c. Block Diagram HU4 and its explanation:

![Figure 9: User Story Number 4 Block Diagram](https://github.com/raul2222/aztec_ws/blob/main/src/aztec_robot/src/images/figure9.png)
Figure 9: User Story Number 4 Block Diagram

The professional reviews the previous therapy sessions stored in Rosbag2 and also schedules new events in the calendar for the child. Aztec, for its part, informs the child about the calendar events through its voice and the Neopixel panel.

d. Flow Diagram HU4:

![Figure 10: User Story Number 4 Flow Diagram](https://github.com/raul2222/aztec_ws/blob/main/src/aztec_robot/src/images/figure10.png)
Figure 10: User Story Number 4 Flow Diagram


Aztec first checks the calendar to verify if there are scheduled tasks. If there are any, Aztec informs the user. If the user decides not to perform the task, Aztec records this decision and returns to the starting point, remaining on standby until the time of the next task comes. On the other hand, if the user accepts the task, Aztec remains on standby until the user finishes, records the completed task, and then returns to the starting point to wait until the time of the next task.

## 4 - Tests

### 5.1 User Story Number 1

The following link shows the first four minutes of 'life' for Aztec. In it, you can appreciate a completely random conversation it has with me. As can be seen in the video, its responses are aligned with the context it comes with by default:


Click on the image below to watch the video:

[![Aztec Robot](https://github.com/raul2222/aztec_ws/blob/main/src/aztec_robot/src/images/figure11.png)](http://www.youtube.com/watch?v=7z9Z30O-XMM "AZTEC ROBOT their first 4 minutes of life with gpt-4 inside")


### 5.2 User Story Number 2

The link provided below shows how Aztec navigates autonomously. As demonstrated in the video, after running the script to initialize Aztec, the RVIZ2 program appears, where we can observe how Aztec perceives the real world. Later, I have it move to various positions in the room and perform a test in which it must dodge an obstacle that is not in its path, a task it accomplishes excellently.

Click on the image below to watch the video:

[![Aztec Robot](https://github.com/raul2222/aztec_ws/blob/main/src/aztec_robot/src/images/figure12.png)](http://www.youtube.com/watch?v=5jJdVX_R80I "AZTEC ROBOT US2")



## 6 - Relationship of the Project with the Sustainable Development Goals (SDGs)

This Final Degree Project (TFG) is directly related to several Sustainable Development Goals (SDGs) defined by the United Nations (UN). The relationships with SDG 3 (Health and Well-being), SDG 4 (Quality Education), and SDG 10 (Reducing Inequalities) are shown below.

### 6.1 Relationship with SDG 3: Health and Well-being

The Aztec robot is directly aligned with the third Sustainable Development Goal (SDG 3) established by the United Nations (UN): Health and Well-being. This goal focuses on promoting health in all its aspects, including physical, mental, and emotional.

Aztec contributes to the realization of this goal in several ways. Firstly, by providing interventions and individualized support to children with Autism Spectrum Disorder (ASD), Aztec plays a fundamental role in improving and maintaining their mental and emotional health. Mental health is an essential component of SDG 3, and children with autism often face considerable challenges in this area.

Through a safe, adapted learning environment, Aztec helps improve the emotional well-being of children, reducing anxiety and stress levels, and improving their self-esteem and social skills. Additionally, Aztec adjusts to the individual needs of each child through personalized interventions based on collected data, thus ensuring effective and appropriate care and support.

Secondly, Aztec also promotes physical health by motivating children to participate in physical activities, thus combating sedentariness. This is a critical component of SDG 3. For children with autism, who often experience difficulties with motor skills and coordination, these physical activities can be particularly beneficial, as they not only improve these skills but also help reduce stress and foster self-confidence.

In summary, Aztec supports SDG 3 by not only improving children's physical health but also strengthening their mental and emotional health, showing clear alignment with the goal of SDG 3 to ensure a healthy life and promote well-being.

### 6.2 Relationship with SDG 4: Quality Education

Aztec, the robot developed in this Final Degree Project (TFG), has the essential purpose of enhancing the quality of educational interventions for children diagnosed with Autism Spectrum Disorder (ASD). This goal directly synchronizes with the Sustainable Development Goal number 4 (SDG 4), which advocates for promoting high-quality education.

The interconnection between Aztec and SDG 4 is reinforced through the use of the TEACCH method in the specific pedagogical techniques implemented in Aztec. This method is designed to provide personalized education that fits the specific needs and abilities of each child.

Moreover, Aztec integrates advanced technologies like natural language processing and machine learning. These advances allow Aztec to provide a more adapted and effective education to each individual. Thus, Aztec contributes to achieving one of the fundamental goals of SDG 4: "ensuring that all young people and a substantial proportion of adults, both men and women, are literate and have acquired basic life and work skills".

### 6.3 Relationship with SDG 10: Reduction of Inequalities

This Final Degree Project (TFG) is in line with Sustainable Development Goal 10 (SDG 10): Reduction of Inequalities, which promotes equity within societies. By developing an educational intervention tool that is both accessible and customizable for children with Autism Spectrum Disorder (ASD), Aztec is leveling educational opportunities.

Early and effective intervention can enable children with ASD to improve their communication and social skills, which can translate into greater social inclusion and more opportunities throughout their lives. By making these interventions more accessible and efficient, Aztec aligns with the SDG 10 goal that seeks to "empower and promote social, economic, and political inclusion of all, regardless of age, sex, disability, race, ethnicity, origin, religion, or economic or other status".

In addition, Aztec has been designed with a focus on sustainability, leveraging second-hand components and recycled plastic to build its chassis. This approach not only supports the circular economy but also reduces the production cost of the robot, facilitating its acquisition and use by a wide range of educational institutions and families.

In conclusion, this project supports the SDGs by addressing both the specific educational needs of children with ASD and the imperative to reduce inequalities and ensure inclusive and quality education for all.

## 6 - User Feedback

A mother of a child with autism and Obsessive-Compulsive Disorder (OCD) provided her feedback after watching a video of Aztec's first four minutes of "life". According to her, it is crucial to reduce the latency time in Aztec's verbal responses, as a prolonged response time could be distressing for her child. In response to this valuable input, it was decided that Aztec would perform inferences on runpod.io using the CUDA cores of an NVIDIA A100.

However, I anticipate that it may be challenging to achieve a response time below five seconds. Although there is always the possibility of being wrong, to date it seems that reducing the response time beyond this limit is an insurmountable challenge with current technology.

Despite this limitation, I have devised some strategies to improve the interaction experience with Aztec. For example, we could display various animations on the neopixel panel and in Aztec's eyes to distract the user during the waiting time. In this way, we could lessen the feeling of waiting and make the interaction with Aztec more pleasant.

## 7 - Conclusions and Future Work

In this Final Degree Project, we have undertaken an initial analysis of Aztec's effectiveness, our robot, in therapies with children with autism. Although there are certain limitations, mainly related to data protection, the initial results are encouraging.

In relation to future work, it will be necessary to resume the trials and collect more data to perform a deeper analysis of Aztec's effectiveness. It will also be important to adjust Aztec's functionalities to better adapt to the individual needs of each child.

At this point, I would like to highlight an open-source project by NVIDIA called Voyager [34]. Voyager is a continuously learning agent in Minecraft powered by long language models like GPT-4, which explores and learns autonomously, accumulates a set of complex skills, and improves itself based on environmental feedback. It interacts with GPT-4 efficiently and its skills are understandable and improvable, allowing for rapid development and avoiding the problem of forgetting previously learned information. Although its application is limited to Minecraft, its implementation in the real world could be interesting.

In summary, this Final Degree Project has been a first approach to exploring the potential of robotics in interventions for autism. Although there is still a long way to go, I am full of optimism and confident that future efforts can continue to build upon this foundation to improve the quality of life for children with autism.


## 8 - References


[1] E. Schopler, S. Brehm, M. Kinsbourne, and R. Reichler, "Effect of Treatment Structure on Development in Autistic Children," Archives of General Psychiatry, vol. 24, no. 5, pp. 415–421, 1971.

[2] American Psychiatric Association. (2013). Diagnostic and Statistical Manual of Mental Disorders (5th ed.). Arlington, VA: American Psychiatric Publishing.

[3] Centers for Disease Control and Prevention. (2020). Autism Spectrum Disorder (ASD). CDC, February. [Online]. Available: https://www.cdc.gov/ncbddd/autism/data.html.

[4] E. Schopler, G. Mesibov, and K. Hearsey, "Structured teaching in the TEACCH system," in Learning and cognition in autism, Springer, 1995, pp. 243-268.

[5] S. Ozonoff and K. Cathcart, "Effectiveness of a home program intervention for young children with autism," Journal of Autism and Developmental Disorders, vol. 28, no. 1, pp. 25–32, 1998.

[6] J. Diehl, L. Schmitt, M. Villano, and C. Crowell, "The clinical use of robots for individuals with autism spectrum disorders: A critical review," Research in Autism Spectrum Disorders, vol. 6, no. 1, pp. 249–262, 2012.

[7] H. Kozima, C. Nakagawa, and Y. Yasuda, "Interactive robots for communication-care: a case-study in autism therapy," IEEE Transactions on Robotics, vol. 23, no. 5, pp. 1050-1056, 2007.

[8] E. Kim, L. Berkovits, R. Bernier, D. Leyzberg, F. Shic, R. Paul, and B. Scassellati, "Social robots as embedded reinforcers of social behavior in children with autism," Journal of Autism and Developmental Disorders, vol. 43, no. 5, pp. 1038–1049, 2013.

[9] Flexible 32x8 NeoPixel RGB LED Matrix. [Online]. Available: https://www.adafruit.com/product/2294 [Accessed: June 2023].

[10] Luxonis, "OAK-D-PRO," [Online]. Available: https://shop.luxonis.com/products/oak-d-pro?variant=42455252369631 [Accessed: June 2023].

[11] Orbbec Technology Group, "MS200," [Online]. Available: http://en.oradar.com.cn/index/product/index.html?id=2 [Accessed: June 2023].

[12] LG, "LG G4 5.5-inch Display," [Online]. Available: https://www.lg.com/ar/celulares/lg-H815AR [Accessed: June 2023].

[13] Samsung, "INR21700-33J 3270mAh - 6.4A," [Online]. Available: https://www.nkon.nl/rechargeable/li-ion/21700-20700-size/samsung-inr21700-33j-3270mah-3-2a-z-tag.html [Accessed: June 2023].

[14] Tof Sensors, "Solid State Lidar CS20," [Online]. Available: https://www.tofsensors.com/products/soild-state-lidar_cs20 [Accessed: June 2023].

[15] TDK, "INMP441," [Online]. Available: https://invensense.tdk.com/wp-content/uploads/2015/02/INMP441.pdf [Accessed: June 2023].

[16] Intel, "Movidius™ Myriad™ X," [Online]. Available: https://www.intel.es/content/www/es/es/products/details/processors/movidius-vpu/movidius-myriad-x.html [Accessed: June 2023].

[17] Intel, "Core i7-6700," [Online]. Available: https://www.intel.com/content/www/us/en/products/sku/88196/intel-core-i76700-processor-8m-cache-up-to-4-00-ghz/specifications.html [Accessed: June 2023].

[18] Corsair, "2 x 8GB Vengeance LPX," [Online]. Available:  https://www.corsair.com/es/es/p/memory/cmk16gx4m2b3200c16/vengeancea-lpx-16gb-2-x-8gb-ddr4-dram-3200mhz-c16-memory-kit-black-cmk16gx4m2b3200c16	[Accessed: 06, 2023].

[19] Samsung, "NVME 250GB EVO 970 PLUS," [Online]. Available:  https://www.samsung.com/es/memory-storage/nvme-ssd/970-evo-plus-1tb-mz-v7s1t0bw/	 [Accessed: 06, 2023].

[20] OpenAI, "GPT-4," [Online]. Available:  https://openai.com/gpt-4	 [Accessed: 06, 2023].

[21] DIGITAL SERVO, "DS3218MG," [Online]. Available:  https://es.aliexpress.com/item/1943129663.html?spm=a2g0o.order_list.order_list_main.53.68f0194dd1lQlZ&gatewayAdapt=glo2esp  [Accessed: 06, 2023].

[22] Open Source, "LangChain," [Online]. Available:  https://python.langchain.com/docs/get_started/introduction.html  [Accessed: 06, 2023].

[23] Open Source, "RVIZ2," [Online]. Available:  https://turtlebot.github.io/turtlebot4-user-manual/software/rviz.html   [Accessed: 06, 2023].

[24] OpenAI (open-source), "Whisper," [Online]. Available:  https://openai.com/research/whisper    [Accessed: 06, 2023].

[25] Google (open-source), "Tacotron 2," [Online]. Available:  https://ai.googleblog.com/2017/12/tacotron-2-generating-human-like-speech.html     [Accessed: 06, 2023].

[26] Open Source, "ROS 2," [Online]. Available:  https://docs.ros.org/en/humble, https://control.ros.org/, https://navigation.ros.org/  [Accessed: 06, 2023].

[27] Pololu, "19:1 Metal Gearmotor 37Dx68L mm 12V with 64 CPR Encoder (Helical Pinion)," [Online]. Available:  https://www.pololu.com/product/4751	  [Accessed: 06, 2023].

[28] Open Source, "SLAM Toolbox," [Online]. Available:  https://github.com/SteveMacenski/slam_toolbox   [Accessed: 06, 2023].

[29] Open Source, "rosbag2," [Online]. Available:  https://github.com/ros2/rosbag2	   [Accessed: 06, 2023].

[30] WHEELTEC, "Neumático de robot de 65mm," [Online]. Available:  https://item.taobao.com/item.htm?id=675857812223   [Accessed: 06, 2023].

[31] Wholesale of Lianggu Furniture Caster Wheels, "Rueda Carter transparente de 2 pulgadas," [Online]. Available:  https://item.taobao.com/item.htm?id=671296900632 	 [Accessed: 06, 2023].

[32] Star Electronic Technology, "2A lithium battery charging board 12.6V," [Online]. Available:  https://item.taobao.com/item.htm?id=652949639498 	 [Accessed: 06, 2023].

[33] Xianyu (Taobao), "JBL Speaker," [Online]. Available:  https://market.m.taobao.com/app/idleFish-F2e/fish-pc/web/detail.html?id=595237376060	  [Accessed: 06, 2023].

[34] NVIDIA (open-source), "Voyager," [Online]. Available:  https://github.com/MineDojo/Voyager  [Accessed: 06, 2023].

[35] F. Martín, A Concise Introduction to Robot Programming with ROS2, 2022.

[36] J. Newans, Articulated Robotics. Available: https://articulatedrobotics.xyz/ [Accessed: 25 de junio de 2023].

[37] A. Sears-Collins, Automatic Addison. Available: https://automaticaddison.com/	   [Accessed: 25 de junio de 2023].

## 9 - Appendices

Types of trials Aztec could undertake to evaluate its efficacy:

1- Randomized Controlled Trials (RCTs): This is a type of scientific study in which children would be randomly assigned to either receive the Aztec intervention or continue with their standard therapy. Progress in key areas (e.g., social skills, adaptive behavior, etc.) will be measured before and after the intervention to determine if there is any difference.

2- Case Studies: Conduct detailed case studies on individual children's interaction with Aztec. This would allow close observation of the child's reactions and responses to the robot, and document any change in their behavior over time.

3- Interviews and Surveys with Parents and Therapists: Gather information about parents' and therapists' experiences with Aztec. They could be asked about what they like and don't like about the robot, and if they have noticed any change in their child's behavior.

4- Usability and Functionality Tests: Conduct usability and functionality tests to ensure that Aztec operates as expected and is easy to use. This could include testing Aztec's features (e.g., movements, interactivity, etc.) and testing how children interact with these features.

5- Measurement of Interaction Frequency and Duration: Collect data on how much time children spend interacting with Aztec and how they interact with it. This could provide valuable information about how children with autism are most comfortable interacting with the robot.

Source Code:

Algorithm for the proportional, integral, and derivative control hosted in ESP32 number 1: https://github.com/raul2222/AZTEC_ESP32_PID

![Aztec Robot](https://github.com/raul2222/aztec_ws/blob/main/src/aztec_robot/src/images/figure13.png)

The following bash script is a key component for its operation, as it starts and manages all of Aztec's autonomous capabilities.

You can find everything you need to fully understand how Aztec works at this link: https://github.com/raul2222/aztec_ws

###!/bin/bash
cd /home/your-user/aztec_ws/
####  Run the first launch file in a new terminal window
gnome-terminal -- bash -c "ros2 launch aztec_robot launch_robot.launch.py; exec bash"
#### Wait a bit for the first node to start working
sleep 3
#### Run the second launch file in a new terminal window
gnome-terminal -- bash -c "ros2 launch oradar_lidar ms200_scan.launch.py; exec bash"
sleep 3
gnome-terminal -- bash -c "ros2 launch aztec_robot localization_launch.py map:=/home/raul/aztec_ws/save.yaml use_sim_time:=false; exec bash"
sleep 3
gnome-terminal -- bash -c "rviz2; exec bash"
#### Set the robot's position from RVIZ2
sleep 16
gnome-terminal -- bash -c "ros2 launch aztec_robot navigation_launch.py use_sim_time:=false map_subscribe_trasient_local:=true; exec bash"
sleep 1
gnome-terminal -- bash -c "ros2 run aztec_robot minodo; exec bash"


