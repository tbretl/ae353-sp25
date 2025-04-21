---
title: DP4 (Drone)
parent: Projects
nav_order: 4
---

# Design Project 4 (racing drone)
{: .no_toc }

- TOC
{:toc }

## System

The fourth project that you will complete this semester is to design, implement, and test a controller that enables a quadrotor - aka "the drone" - to race through rings from start to finish without crashing.

![Image of drone](images/drone.png)

In particular, your drone will be racing with other drones. You will need to take care not to run into these other drones, as collisions may cause drones to crash and cat-pilots to be lost.

Keep your cat-pilot safe! Give them victory!


## Context {#drone-context}

Imagine that, working as a control systems engineer, you have been hired by [Tiny Whoop](https://www.tinywhoop.com/) to design a controller for a small-scale drone. In particular, imagine that Tiny Whoop intends to market this drone to amateur cat-pilots for racing (e.g., see their [alley cat coffee cup invitational race](https://youtu.be/jnXgoRxyVqU?t=283)). Your job is to show that this drone — with a suitable controller — is capable of high-speed, agile flight.

As you work on this project, we encourage you to think about other possible applications of drones such as these. [PowderBee](https://snowbrains.com/new-avalanche-rescue-drone-in-the-works/) from [Bluebird Mountain](https://www.springwise.com/innovation/sport-fitness/powderbee-drone-avalanche-rescue/), for example, is intended to find avalanche victims (i.e., people buried under thick snow) quickly so they can be rescued before succumbing to asphyxiation, hypothermia, or other injuries — every second counts in rescue applications, just as for racing drones.


## Model

The motion of each drone is governed by ordinary differential equations with the following form:

$$\begin{bmatrix} \dot{p}_x \\ \dot{p}_y \\ \dot{p}_z \\ \dot{\psi} \\ \dot{\theta} \\ \dot{\phi} \\ \dot{v}_x \\ \dot{v}_y \\ \dot{v}_z \\ \dot{w}_x \\ \dot{w}_y \\ \dot{w}_z \end{bmatrix} = f\left(p_x, p_y, p_z, \psi, \theta, \phi, v_x, v_y, v_z, w_x, w_y, w_z, \tau_x, \tau_y, \tau_z, f_z \right)$$

In these equations:

* $p_x$ is the **$x$ position** (m)
* $p_y$ is the **$y$ position** (m)
* $p_z$ is the **$z$ position** (m)
* $\psi$ is the **yaw angle** (rad)
* $\theta$ is the **pitch angle** (rad)
* $\phi$ is the **roll angle** (rad)
* $v_x$ is the **linear velocity along the body-fixed $x$ axis** (m/s)
* $v_y$ is the **linear velocity along the body-fixed $y$ axis** (m/s)
* $v_z$ is the **linear velocity along the body-fixed $z$ axis** (m/s)
* $w_x$ is the **angular velocity about the body-fixed $x$ axis** (rad/s), which points forward
* $w_y$ is the **angular velocity about the body-fixed $y$ axis** (rad/s), which points left
* $w_z$ is the **angular velocity about the body-fixed $z$ axis** (rad/s), which points up
* $\tau_x$ is the **net torque about the body-fixed $x$ axis** ($N\cdot\text{m}$)
* $\tau_y$ is the **net torque about the body-fixed $y$ axis** ($N\cdot\text{m}$)
* $\tau_z$ is the **net torque about the body-fixed $z$ axis** ($N\cdot\text{m}$)
* $f_z$ is the **net force along the body-fixed $z$ axis** ($N$)

A [symbolic description of these equations of motion]({{ site.github.repository_url }}/tree/main/projects/04_drone/DeriveEOM-Template.ipynb) is provided with the [project code]({{ site.github.repository_url }}/tree/main/projects/04_drone).

{: .note}
> The body frame attached to the drone has $x$ forward, $y$ **left**, and $z$ **up**. This convention is different from what is commonly used for aircraft (with $y$ right and $z$ down), and in particular is different from what you used in your [second design project](02-zagi). A positive yaw angle means the drone rotates **left** (not right, like the aircraft). A positive pitch angle means the drone rotates **down** (not up, like the aircraft).

The actuators that produce the net torques $\tau_x, \tau_y, \tau_z$ and net force $f_z$ are the four rotors on the drone, each of which is driven by an electric motor. These rotors can spin in only one direction, and have minimum and maximum speeds. These bounds limit the torques and forces that can actually be produced. These limits are more complicated than simple minimums and maximums. Here are two ways to get a sense for what these limits are:

* Call the function

  ```python
  (
    tau_x,
    tau_y,
    tau_z,
    f_z,
  ) = simulator.enforce_motor_limits(
    tau_x_cmd,
    tau_y_cmd,
    tau_z_cmd,
    f_z_cmd,
  )
  ```
  to find the torques and forces that would actually be applied for given torque and force commands. Note that this function cannot be called from within your controller code — it can only be called from elsewhere in your notebook, for the purpose of testing.

* Use your data from simulation to plot both the torque and force commands as well as the torques and forces that are actually applied. An example is provided in the template notebook.

Ask if you want more details about the mapping from commanded to actual forces and torques.

Sensors provide a noisy measurement of the position in space of two markers (`pos_markers`), one at the center of the left rotor and one at the center of the right rotor. This is the sort of measurement that would be provided by a standard commercial motion capture system (e.g., [OptiTrak](https://optitrack.com) or [Vicon](https://www.vicon.com)). A [symbolic description of sensor model]({{ site.github.repository_url }}/tree/main/projects/04_drone/DeriveEOM-Template.ipynb) is provided with the [project code]({{ site.github.repository_url }}/tree/main/projects/04_drone).

In addition to these marker position measurements, your controller is also provided with the following information:

* the position (`pos_ring`) of the center of the next ring that the drone needs to pass through
* the vector (`dir_ring`) normal to this next ring (i.e., pointing through its center)
* a flag (`is_last_ring`) that indicates if this next ring is the last ring, a.k.a., the "finish"
* the position (`pos_others`) of all other drones that are still flying.

The code provided [here]({{ site.github.repository_url }}/tree/main/projects/04_drone) simulates the motion of this system ([DroneDemo]({{ site.github.repository_url }}/tree/main/projects/04_drone/DroneDemo-Template.ipynb)) and also derives the dynamic model and sensor model in symbolic form ([DeriveEOM]({{ site.github.repository_url }}/tree/main/projects/04_drone/DeriveEOM-Template.ipynb)).

The goal is to race as fast as possible from the start ring to the goal ring, passing through all the rings in between.

## Tasks

Do the following things to produce a control design:

* Linearize the dynamic model and the sensor model.
* Show that the linearized system is both controllable and observable.
* Design a stable controller and a stable observer.
* Add trajectory tracking to enable movement between rings.
* Implement the controller and observer (with trajectory tracking) and test them in simulation.

Do the following things to evaluate your control design:

* Identify and diagnose as many sources of failure as you can find.
* Create at least four figures of aggregate results from at least 100 simulations:
  * Show how well your controller is working (e.g., with a histogram of error between actual position and desired position).
  * Show how well your observer is working (e.g., with a histogram of error between estimated position and actual position).
  * Show how fast your drone completes the race (e.g., with a histogram of completion times). You may also want to show how likely it is that your drone completes the race at all, without failure.
  * Show how long it takes your controller to run (e.g., with a histogram of computation times). This will help you avoid being disqualified in the [race](#contest).

Do the following thing to compete in the [race](#contest):

* Submit a control design that runs without being disqualified (i.e., it does not raise an exception, it does not print any debug text, and it does not exceed limits on computation time).

Just as in your previous two design projects, you must be specific about what you mean by "success" and you must provide **quantitative** evidence to support the claim that you have (or have not) succeeded. Remember that people often think about this in terms of **requirements** and **verifications**.

{: .note-title}
> Analysis of failure
>
> In this project, we would like you to focus on identifying and diagnosing failures. In this context, a "failure" is anything that causes your drone not to reach the finish ring. There are many reasons why a particular control design might lead to failure. Your job as a control engineer is to uncover and address these sources of failure. At minimum, you should do the following two things for your final control design:
> 
> * **Identify** as many failures as possible.
> * **Diagnose** each failure, saying why you think it happened and providing evidence to support your claim.
> 
> It is often helpful to put failures into categories — that is, to identify and diagnose *types* of failures, and to use *particular* failures as a way to illustrate each type.
> 
> Remember that you have practice in doing rigorous data collection, analysis, and visualization (the focus of the [third design project](#design-project-3-spacecraft-with-star-tracker)). Rigorous data collection can help a lot in identifying and diagnosing failures.
> 
> You may, of course, be tempted to **eliminate** failures after you find them, by making some change to your control design. This is exactly the right thing to do, but remember that after you make a change, you'll have to repeat (completely) your analysis of failure.

In doing these things, **keep your focus on the safety and well-being of your cat-pilot**. They need to know not only that your control system is reliable, but also what failures are possible and how likely they are to occur.


## Deliverables

### Draft report with theory (by 11:59pm on Friday, April 18) {#first-draft}

Submit a first draft of your [report (see below for guidelines)](#final-report). This draft must include a complete *Theory* section, which is what we will focus on in our review. This draft must also include the *Abstract*, *Nomenclature*, and *Introduction*.

Upload it to the [**DP4 Draft 1** group assignment on Canvas](https://canvas.illinois.edu/courses/54818/assignments/1225818).


### Draft report with results (by 11:59pm on Friday, April 25) {#second-draft}

Submit a second draft of your [report (see below for guidelines)](#final-report). This draft must include a complete *Experimental methods* section and a complete *Results and discussion* section, which are what we will focus on in our review. This draft must also include the *Conclusion*, *Appendix*, *Acknowledgements*, and *References*.

Upload it to the [**DP4 Draft 2** group assignment on Canvas](https://canvas.illinois.edu/courses/54818/assignments/1225819).


### Contest entry (by 5:00pm on Tuesday, May 6) {#contest-code}

There will be an opportunity to race with your friends in a friendly contest on the last day of class (Wednesday, May 7). The [same project code]({{ site.github.repository_url }}/tree/main/projects/04_drone) that you are using for the purpose of control design and simulation will be used to run each race in this contest. To enter the race, your team must upload exactly two files to [the "DP4-Race-Submissions" Box folder](https://uofi.box.com/s/evg9tr11zrtymotel1ft6zs6mu66bkxh):

* `netid.py`, with a completely self-contained implementation of your control design, in the format specified by [04_drone/students/template.py]({{ site.github.repository_url }}/tree/main/projects/04_drone/students/template.py).
* `netid.png`, with an image (keep it professional) that can be used to distinguish your drone from others.

You must, of course, replace `netid` with your own netid. Please use all lowercase.

**Please submit only one pair of files - with the NetID of only one group member - for each group.** Your submission will automatically be associated with all members of your group.

**All groups are required to enter the race!** Your submission will be assessed based on whether or not it satisfies three conditions:

* It must not raise an exception (i.e., throw an error).
* It must not print anything to `stdout` (i.e., run `print` statements).
* It must not exceed limits on computation time (5 seconds for module load, 1 second for `__init__`, 1 second for `reset`, and `1e-2` seconds for `run`).

Your submission will *not* be assessed based on whether or not it wins the race.


### Final report (by 11:59pm on Tuesday, May 6) {#final-report}

This report will satisfy the following requirements:

* It must be a single PDF document that conforms to the guidelines for [Preparation of Papers for AIAA Technical Conferences](https://www.aiaa.org/events-learning/events/Technical-Presenter-Resources). In particular, you **must** use either the [Word](https://www.aiaa.org/docs/default-source/uploadedfiles/aiaa-forums-shared-universal-content/preparation-of-papers-for-technical-conferences.docx?sfvrsn=e9a97512_10) or [LaTeX](https://www.overleaf.com/latex/templates/latex-template-for-the-preparation-of-papers-for-aiaa-technical-conferences/rsssbwthkptn#.WbgUXMiGNPZ) manuscript template.
* It must have a descriptive title that begins with "DP4" (e.g., "DP4: Control of a racing drone").
* It must have a list of author names and affiliations.
* It must contain the following sections:
  * *Abstract.* Summarize your entire report in one short paragraph.
  * *Nomenclature.* List all symbols used in your report, with units.
  * *Introduction.* Prepare the reader to understand the rest of your report and how it fits within a broader context.
  * *Theory.* Derive a model and do control design.
  * *Experimental methods.* Describe the experiments you performed in simulation in enough detail that they could be understood and repeated by a colleague.
  * *Results and discussion.* Show the results of your experiments in simulation (e.g., with plots and tables) and discuss the extent to which they validate your control design and support an argument for the safety of your racing system.
  * *Conclusion.* Summarize key conclusions and identify ways that others could improve or build upon your work.
  * *Appendix.* Provide a review of your racing system from the perspective of one or more of your cat-pilots. They are important stakeholders. You are welcome to refer to this appendix — i.e., to the remarks from your cat-pilots — in other parts of your report, if it is helpful in supporting your arguments.
  * *Acknowledgements.* Thank anyone outside your group with whom you discussed this project and clearly describe what you are thanking them for.
  * *References.* Cite any sources, including the work of your colleagues.
* It must contain a URL (with hyperlink) to your [final video](#final-video).
* It must be a maximum of 6 pages.

Submit your report by uploading it to the [**DP4 Report** group assignment on Canvas](https://canvas.illinois.edu/courses/54818/assignments/1225821).

### Final video (by 11:59pm on Tuesday, May 6) {#final-video}

This video will satisfy the following requirements:

* It must be 60 seconds in length.
* The first and last 5 seconds must include text with a descriptive title (the same title as your report), your names, and the following words somewhere in some order:
  * AE353: Aerospace Control Systems
  * Spring 2025
  * Department of Aerospace Engineering
  * University of Illinois at Urbana-Champaign
* The middle 50 seconds must communicate the highlights of your methods and results to potential stakeholders. Who are you designing for? How will your controller help them?
* It must show at least one simulation of your working control system.
* It must include at least one remark from a cat-pilot.
* It must be engaging (have fun with this).
* It must not be offensive --- use common sense and ask if you are uncertain.

Submit your video by uploading it to the [AE353 (Spring 2025) Project Videos](https://mediaspace.illinois.edu/channel/channelid/369105032) channel on Illinois Media Space. Please take care to do the following:

* Use the same descriptive title as your report, appended with your names in parentheses --- for example, "DP4: Control of a racing drone (Tim Bretl and Jacob Kraft)".
* Add the tag `dp4` (a **lower case** "dp" followed by the number "4"), so viewers can filter by project number.
* Ask someone else to confirm that they can view the video on Media Space (i.e., that it has, indeed, been published to the correct channel).

You are welcome to resubmit your video at any time before the deadline. To do so, please "Edit" your **existing** video and then do "Replace Media". Please do **not** create a whole new submission.

We realize that 60 seconds is short! Think carefully about what to include (what to show and what to say) and anticipate the need for multiple "takes" and for time spent editing.

Please also submit the URL for your video to the [**DP4 Video** group assignment on Canvas](https://canvas.illinois.edu/courses/54818/assignments/1225822). You can find this URL by viewing your video on Media Space and then by clicking "Share" and "Link to Media Page".


### Final code (by 11:59pm on Tuesday, May 6) {#final-code}

This code will satisfy the following requirements:

* It must be a single jupyter notebook (with the extension `.ipynb`) that, if placed in the `projects/03_spacecraft` directory and run from start to finish, would reproduce *all* of the results that you show in your report.
* It must not rely on any dependencies other than those associated with the [`ae353` conda environment](../setup).
* It must be organized and clearly documented, with a mix of markdown cells and inline comments.

Submit your code by uploading it to the [**DP4 Code** group assignment on Canvas](https://canvas.illinois.edu/courses/54818/assignments/1225817). You will be asked to upload it in two formats — as the original `.ipynb` (so we can run your code) and as rendered `.html` (so we can see and comment on your code in Canvas). Follow [these instructions](https://code.visualstudio.com/docs/datascience/jupyter-notebooks#_export-your-jupyter-notebook) to get your notebook in `.html` format.


### Individual reflection (by 11:59pm on Wednesday, May 7)

Complete the [**DP4 Reflection** assignment on Canvas](https://canvas.illinois.edu/courses/54818/assignments/1225820) sometime between 11:59pm on Sunday, May 4 and 11:59pm on Wednesday, May 7. This assignment, which should take no more than 10 or 15 minutes, will give you a chance to reflect on your experiences during this project and throughout the semester.


## Evaluation

Your project grade will be weighted as follows:

* (10%) Draft report with theory
* (10%) Draft report with results
* (40%) Final report
* (20%) Final video
* (10%) Final code and contest entry
* (10%) Individual reflection

Rubrics will be discussed in class.
