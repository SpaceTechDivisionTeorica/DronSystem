# MIT xPro — Architecture of Complex Systems

This folder contains my projects and work from the MIT xPro program 
on Architecture and Systems Engineering. The program is built around 
a pretty fundamental idea: the world runs on increasingly complex 
systems, and engineers need better ways to design, manage, and 
make decisions about them.

Throughout the course we explored how to break down complex systems 
into their components, understand how they interact, and make 
architectural decisions that actually hold up under pressure. A big 
part of the thinking is learning to treat architecture as a series 
of decisions that can be sorted, evaluated, and optimized rather 
than something fixed or intuitive.

We also worked with Model-Based Systems Engineering (MBSE), which 
is about using models not just as documentation but as active tools 
to analyze, validate, and communicate how a system behaves. It 
shifts the way you think about engineering problems from reactive 
to structured.

The industries where this kind of thinking applies are broad, 
aerospace, automotive, defense, high tech, but the core skill is 
really about thinking holistically. How do you see the whole system, 
not just your piece of it?

That is what this folder is about. Work in progress, lessons learned, 
and a way of thinking I am still developing.

## Project: Develop and Analyze — Anduril's Pulsar (Electromagnetic Warfare)

My project for the Develop and Analyze module applies the course's 
architectural decision framework to a real system: **Pulsar**, an 
Electromagnetic Warfare (EW) product built by **Anduril Industries**.

### About Anduril Industries

Anduril is a US defense technology startup founded in 2017 by Palmer 
Luckey (co-founder of Oculus VR), Trae Stephens, Matt Grimm, Brian 
Schimpf, and Joseph Chen. Unlike the traditional defense primes 
(Lockheed Martin, Raytheon, Northrop Grumman), Anduril operates as a 
venture-backed product company: it builds hardware and autonomous 
systems in-house, self-funds development ahead of contracts, and 
iterates the way a software company would rather than relying purely 
on cost-plus government contracting. Its product line includes 
autonomous sentry towers, counter-UAS systems, autonomous underwater 
and surface vehicles, loitering munitions, and **Lattice**, its 
AI-driven command-and-control software that fuses sensor data across 
a battlespace into a shared operating picture.

### Why Pulsar

Pulsar is Anduril's compact, software-defined Electromagnetic Warfare 
system, built to detect, track, and defeat drone and other 
RF-enabled threats. It's a useful case study for this course because 
it sits exactly at the intersection this program is about: hardware, 
software, and mission-systems architecture combined into one 
"system of systems." Studying it means working through the same 
tradeoffs the course frames as architectural decisions — sensor and 
effector tradeoffs, integration with broader C2 systems like 
Lattice, spectrum and interoperability constraints, and how early 
architectural choices propagate downstream into operational 
effectiveness.

The goal of this project is to apply the architectural decision 
framework from the course — decision identification, alternative 
generation, evaluation criteria, and tradeoff analysis — to Pulsar's 
design space, and document how those decisions hold up under the 
kind of pressure real defense systems are built for.
