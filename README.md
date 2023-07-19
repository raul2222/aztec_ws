# UNIVERSITAT POLITÈCNICA DE VALÈNCIA
## School of Gandia Superior Polytechnic
### Development of Aztec: An experimental robot for intervention in children with autism using the TEACCH method
#### Final Degree Project 
#### Degree in Interactive Technologies
##### AUTHOR: Santos Lopez, Raul 
##### Tutor: Pérez Pascual, Mª Asunción 
##### ACADEMIC YEAR: 2022/2023

## Index
1. [Introduction](#introduction)  
2. [Objectives](#objectives)  
    2.1 [Main Objective](#main-objective)  
    2.2 [Secondary Objectives](#secondary-objectives)  
3. [Methodology and Technologies used](#methodology-and-technologies-used)  
    3.1 [Description of the Aztec Robot](#description-of-the-aztec-robot)  
        3.1.1 [Appearance and general design](#appearance-and-general-design)  
    3.2 [Proposed methodology to work with the robot](#proposed-methodology-to-work-with-the-robot)  
        3.2.1 [TEACCH Method](#teacch-method)  
        3.2.2 [Data collection](#data-collection)  
        3.2.3 [Configuration of Aztec for therapy sessions](#configuration-of-aztec-for-therapy-sessions)  
        3.2.4 [Evaluation and improvement](#evaluation-and-improvement)  
    3.3 [Technologies used to implement the robot](#technologies-used-to-implement-the-robot)  
        3.3.1 [Hardware](#hardware)  
        3.3.2 [Software](#software)  
    3.4 [Implementation of the Aztec robot](#implementation-of-the-aztec-robot)  
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

| Name | Description |
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

User Story Number 1

a. Description of US1: 

"As the mother of a child with ASD, I want Aztec to be able to verbally interact with my child to help him improve his communication skills."

b. Algorithms and processes used to implement US1:

| Process | Description |
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

Figure 4: Flow Diagram User Story number 1

![Aztec Robot](https://github.com/raul2222/aztec_ws/blob/main/src/aztec_robot/src/images/figure4.png)

If the system detects an absence of voice for 2 seconds after previously detecting voice, it sends the buffer to Whisper. If the transcription's language is not the expected one, it's discarded. Otherwise, it's sent to Langchain. Langchain processes the text and sends it to GPT-4. The response from GPT-4 is sent both to Langchain and Tacotron 2. Tacotron 2, in collaboration with MelGAN, synthesizes the text into voice, which is played through the speakers.

User Story number 2

a. Description of User Story 2 (US2):

"As an occupational therapist and educator specialized in ASD, I wish Aztec can recognize, adapt, and move autonomously and safely in its environment. This will allow Aztec to effectively interact with children and students during therapy sessions and in the classroom without the need for constant supervision."

b. Algorithms and processes used to implement US2:



