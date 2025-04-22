---
title: MacOS
parent: Setup
---

# MacOS
{: .no_toc }

- TOC
{:toc }

## Do these things once at the start of the semester

### Install xcode command line tools

Open a terminal and run this command, accepting all default options:

```
xcode-select --install
```

You may be asked to restart your computer during or after this process. Please do so.


### Install conda

[Conda](https://docs.conda.io/) is a package manager that you will use to install python, along with a number of required tools.

Unlike many other applications on your computer, it is possible to install different versions of Conda in three different ways (with Anaconda, Miniconda, or Miniforge), all at the same time. This can be confusing and can lead to all sorts of trouble. So, unless you know what you are doing, we very strongly recommend two things:

* **Do not** use Anaconda (so, uninstall it before doing anything else).
* Use **either** Miniconda or Miniforge to install Conda **exactly once**.

First, to check if Anaconda is installed, open a terminal and run the following command:

```
which anaconda-navigator
```

If you see anything other than `anaconda-navigator not found`, then anaconda is installed. Follow these instructions (including the optional steps to remove "conda initialization scripts" and to remove "hidden directories") to uninstall it:

* [Uninstall Anaconda Distribution](https://docs.anaconda.com/anaconda/install/uninstall/)


{: .note-title}
> How to open a terminal
>
> When we say "open a terminal," what we mean is to start the **Terminal** application. Here is one way to do that:
> 
> * Click the Launchpad icon in the Dock.
> * Type "Terminal" in the search field.
> * Click Terminal.
> 
> See documentation on [Open Terminal](https://support.apple.com/guide/terminal/open-or-quit-terminal-apd5265185d-f365-44cb-8b09-71a064a42125/mac) for more information. Note that it is often helpful to have more than one terminal window open at the same time (or more than one tab in the same window).

{: .note-title}
> How to run a command
> 
> When we say "run a command," what we mean is to type something into the terminal window and press return. For example, suppose we said:
> 
> > run the command `pwd` to find your current working directory
> 
> You would type `pwd` into the terminal window and press return, with the result being something like this:
> 
> ```
> (base) timothybretl@Timothys-MacBook-Pro ~ % pwd
> /Users/timothybretl
> ```
> 
> See documentation on [Execute commands and run tools in Terminal on Mac for more information](https://support.apple.com/guide/terminal/execute-commands-and-run-tools-apdb66b5242-0d18-49fc-9c47-a2498b7c91d5/mac). Also see the [Command Line Primer](https://developer.apple.com/library/archive/documentation/OpenSource/Conceptual/ShellScripting/CommandLInePrimer/CommandLine.html) for a list of frequently used commands.

Second, to check if either Miniconda or Miniforge is installed, open a terminal and run the following command (it tries to update whatever is installed --- Miniconda or Miniforge --- to its latest version, something you should do from time to time anyway):

```
conda update -n base conda
```

If this process succeeds, then either Miniconda or Miniforge is installed.

Third, to install Miniforge (do this *only* if neither Miniconda nor Miniforge was installed!), follow [these instructions](https://github.com/conda-forge/miniforge?tab=readme-ov-file#unix-like-platforms-macos--linux), which require only running the following commands in a terminal:

```
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh -b -p "${HOME}/miniforge3"
rm Miniforge3-$(uname)-$(uname -m).sh
~/miniforge3/bin/conda init bash
~/miniforge3/bin/conda init zsh
```

Please check very carefully (by reading the output in the terminal) that all **five** of these commands ran to completion without producing any errors. If you see any errors, ask us for help before doing anything else.

After running these commands (error-free!), close your terminal and open a new one before proceeding to the next step.

### Get the code

Open a terminal and change the working directory to a folder in which you would like to put all your files. Then, use [git](https://git-scm.com/) to download the code from our [ae353 github repository]({{ site.github.repository_url }}) by running this command:

```
git clone https://github.com/tbretl/ae353-sp25.git
```

This process will take very little time. When it completes, you should find a new folder called `ae353-sp25` inside your working directory.

{: .note-title}
> How to change the working directory
> 
> All the files on your computer are organized in folders, which are commonly referred to as "directories." When you are working on the command line in a terminal, you are working in one of these directories. Commands you run can find files in that directory, but cannot (by default) find files in other directories.
> 
> When we say "change the working directory," we mean exactly that --- telling the terminal the directory in which you want to work.
> 
> To do this, we run the command
> 
> ```
> cd path/to/directory
> ```
> 
> where "`path/to/directory`" is replaced by the location of the directory in which you want to work. One easy way to find this location (i.e., the "path" to your directory) is by dragging its folder from the Finder into your terminal window (see documentation on [Drag items into a Terminal window on Mac](https://support.apple.com/guide/terminal/drag-items-into-a-terminal-window-trml106/mac)). In particular, I would first type "`cd `" (note the single trailing space):
> 
> ```
> (base) timothybretl@Timothys-MacBook-Pro ~ % cd 
> ```
> 
> Then, I would drag a folder into the terminal window and press return. For instance, suppose I had created a folder called `ae353-sp25` somewhere on my computer and dragged it in, then pressed return --- I would see something like this:
> 
> ```
> (base) timothybretl@Timothys-MacBook-Pro ~ % cd /Users/timothybretl/Documents/ae353-sp25
> (base) timothybretl@Timothys-MacBook-Pro ae353-sp25 %
> ```
> 
> See documentation on [Specify files and folders in Terminal on Mac](https://support.apple.com/guide/terminal/specify-files-and-folders-apd3cf6fe02-3ec8-48f1-951f-866e52955fc8/mac) for other ways to specify the path to a directory.



### Create a conda environment

An "environment" is like a sandbox where you can install software without causing any conflict with other things you might have installed on your computer. To create an environment for work in AE353, change your working directory to `ae353-sp25` (wherever you put the code). Then, run this command:

```
conda env create -f environment.yml
```

You may see something like `WARNING: A conda environment already exists` and be asked if you want to `Remove existing environment`. This means that you already created an environment called `ae353`. Perhaps something went wrong and you are creating it again --- in this case, type `y` to remove the existing environment and proceed to recreate it. From then on, accept all default options.

Make sure that this process completes successfully. If you see any errors, ask for help.

### Install VS Code

[Visual Studio Code](https://code.visualstudio.com/) (aka VS Code) is the cross-platform code editor that we recommend for use with this course.

To install VS Code, follow the instructions for [Installation](https://code.visualstudio.com/docs/setup/mac#_installation) (being sure to remember to **drag VS Code into your Applications folder**) and for [Launching from the command line](https://code.visualstudio.com/docs/setup/mac#_launching-from-the-command-line) from the [Visual Studio Code on macOS](https://code.visualstudio.com/docs/setup/mac) setup page. 

To run VS Code, open a terminal, change the working directory to wherever you have your files, and run the command `code .` (i.e., the word "code", then a space, then a period).

This semester, you will primarily be using VS Code to work with jupyter notebook files (`.ipynb`). This requires installing the [Python Extension](https://marketplace.visualstudio.com/items?itemName=ms-python.python) and the [Jupyter Extension](https://marketplace.visualstudio.com/items?itemName=ms-toolsai.jupyter) to VS Code. To do this, open VS Code if you haven't already, then click on **Extensions** in the toolbar on the left side of the window.

Install the python extension as follows:

* Type "python" into the search bar.
* Click on **Python** (it should be at the top of the list).
* Click on **Install** in the window that opens.

Install the jupyter extension as follows:

* Type "jupyter" into the search bar.
* Click on **Jupyter** (it should be at the top of the list).
* Click on **Install** in the window that opens.


## Do these things every day before you start your work

### Change your working directory

Open a terminal and change your working directory to `ae353-sp25`, wherever you put this.

### Activate your conda environment

Run this command:

```
conda activate ae353
```

You should see the prefix to your terminal prompt change from `(base)` to `(ae353)`. This means you are in the conda environment you created for work with AE353.

### Get the latest version of the code

Run this command:

```
git pull
```

Do not worry, this will not overwrite any of your own work. If you see any errors or warnings, post a note to [the discussion forum on canvas](https://canvas.illinois.edu/courses/54818/discussion_topics) and course staff will help resolve them.


### Start a jupyter notebook

Run this command to open VS Code:

```
code .
```

You can now open and work with any notebook (or other file) that you like.

As an alternative to VS Code, if you like, you can also work with jupyter notebooks in a browser window. To do so, and run this command:

```
jupyter notebook
```

A browser window should open with the jupyter notebook interface. You can now navigate to and open any of the notebooks (with extension `.ipynb`) used for in-class examples or for design projects.

In either case, **we strongly recommend you duplicate and work with a copy of any given notebook rather than working with the original.** Feel free to ignore this suggestion if you are a `git` expert.