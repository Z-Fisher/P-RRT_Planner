# P-RRT_Planer

MEAM 520
Final Project Code
Team 10: Zachary Fisher & Michael Woc

Instructions:
Please open "wrapper.m" and run the code. Some parameters can be changed in the code to vary the environment description. Some commented out sections of the code were used for specific experiments and so this version of the code works for the main focus of our report (dynamic obstacles only).



December 9th, 2019

Most of my work was in developing our path-planning, simulation, and experimentation code. We were both partners for the Path Planning and Potential Field Labs and so we each had a strong understanding of that material as well as some mutual leftover code that acted as a guide. I first remade the RRT and Potential Field planners to work in a 2D environment for a holonomic vehicle. This also involved developing a new method of defining and simulating the environment now that we were no longer using the Lynx. It took more effort than expected to simply convert these two planners to our new scenario; in fact, representing them this way revealed some flaws in the algorithms that were not as easily noticeable in the original 3D Lynx simulator provided to us. I made multiple improvements outlined in the report to optimize their performance for the 2D cases we explored. I then worked on improvising a version of a potential field planner that is guided by RRT instead of just following a pseudocode description from a research paper. In addition to the bulk of the programming, I wrote the “Methods” and “Conclusions & Future Work” sections of our final report and created the all graphical images and videos used to represent our experiments.

I did not expect to spend as much time and effort on simulating our results as I did. It detracted from focusing purely on the path planning objectives and in the future, I will likely look for open-source resources first to offload this labor. While developing the simulations was tedious and required heavy debugging, the most technically challenging aspect of this project was in creating the logical flow for P-RRT. My first approach was a complete failure as I found I had not generalized the code sufficiently and was moving towards a recursive approach, with which I do not have experience. The issue forced me to restructure the potential field and RRT functions so that they could each be called from a single line. The P-RRT pseudocode in my group’s report outlines nicely the cleanliness of the structure with this approach.

Path planning has become my favorite topic thus far as a robotics student. My background in mechanical engineering made me nervous at first when I was introduced to the topic due to the role of programming. I really enjoy the logic and methodology behind them and find it interesting to break down a task that would otherwise be straightforward for a human into a programmable description.