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
    1. [Main Objective](#main-objective)
    2. [Secondary Objectives](#secondary-objectives)
3. [Methodology and Technologies used](#methodology-and-technologies-used)
    1. [Description of the Aztec Robot](#description-of-the-aztec-robot)
        1. [Appearance and general design](#appearance-and-general-design)
    2. [Proposed methodology to work with the robot](#proposed-methodology-to-work-with-the-robot)
        1. [TEACCH Method](#teacch-method)
        2. [Data collection](#data-collection)
        3. [Configuration of Aztec for therapy sessions](#configuration-of-aztec-for-therapy-sessions)
        4. [Evaluation and improvement](#evaluation-and-improvement)
    3. [Technologies used to implement the robot](#technologies-used-to-implement-the-robot)
        1. [Hardware](#hardware)
        2. [Software](#software)
    4. [Implementation of the Aztec robot](#implementation-of-the-aztec-robot)
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

![Aztec Robot](https://raw.githubusercontent.com/raul2222/aztec_ws/src/images/figure1.png)


Figure 1. Photograph of Aztec robot


