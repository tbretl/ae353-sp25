---
title: Windows
parent: Setup
---

# Windows
{: .no_toc }

- TOC
{:toc }

## Do these things once at the start of the semester

### Install conda

[Conda](https://docs.conda.io/) is a package manager that you will use to install python, along with a number of required tools.

Unlike many other applications on your computer, it is possible to install different versions of Conda in three different ways (with Anaconda, Miniconda, or Miniforge), all at the same time. This can be confusing and can lead to all sorts of trouble. So, unless you know what you are doing, we very strongly recommend two things:

* **Do not** use Anaconda (so, uninstall it before doing anything else).
* Use **either** Miniconda or Miniforge to install Conda **exactly once**.

First, to check if Anaconda is installed, search for "Anaconda Navigator" from the **Start** menu. If you find it, then follow these instructions to uninstall it:

* [Uninstall Anaconda Distribution](https://docs.anaconda.com/anaconda/install/uninstall/)

Second, to check if Miniconda is installed, search for "Anaconda Prompt" from the **Start** menu. If you find it, then open it and confirm that you see the text `(base)` on the command line. If you do, then Miniconda is installed. Run the following command to update Miniconda to its latest version (something you should do from time to time anyway):

```
conda update -n base conda
```

Third, to check if Miniforge is installed (do this *only* if Miniconda was not installed!), search for "Miniforge Prompt" from the **Start** menu. If you find it, then open it and confirm that you see the text `(base)` on the command line. If you do, then Miniforge is installed. Run the following command to update Miniforge to its latest version (something you should do from time to time anyway):

```
conda update -n base conda
```

Fourth, to install Miniforge (do this *only* if neither Miniconda nor Miniforge was installed!), follow these instructions:

* [Install Miniforge](https://github.com/conda-forge/miniforge/?tab=readme-ov-file#windows)

{: .note-title}
> What to do if you get a "dangerous site" warning
>
> If you are using Chrome to download Miniforge, you may get a "dangerous site" warning. Do the following things to bypass this warning:
> * Click **More &vellip; > Downloads** in the top right corner of the browser window.
> * Find the file you want to download.
> * Click either **More &vellip; > Download dangerous file** (according to [google](https://support.google.com/chrome/answer/99020?hl=en&co=GENIE.Platform%3DDesktop#zippy=%2Cdownload-an-unsafe-file)) or **Keep dangerous file > Keep anyway** (according to [qwiqode](https://qwiqode.com/blogs/qwiq-tips-tricks/how-to-stop-google-chrome-from-blocking-your-download)).

### Install git

Open a terminal.

{: .note-title}
> How to open a terminal
>
> When we say "open a terminal," what we mean is to do one of two things:
>
> * If **Miniconda** is installed, then use the **Start** menu to search for and open an "Anaconda Prompt."
> * If **Miniforge** is installed, then use the **Start** menu to search for and open a "Miniforge Prompt."
>
> There may be other words in the titles of these two applications — for example, "Anaconda Prompt" may appear as "Anaconda Prompt (Miniconda3)."

Verify that you see the text `(base)` on the command line in the terminal. Run the following command to install [git](https://git-scm.com/):

```
conda install git -y
```

### Get the code

Open a terminal and change the working directory to a folder in which you would like to put all your files.

{: .note-title}
> How to change the working directory
>
> All the files on your computer are organized in folders, which are commonly referred to as "directories." When you are working on the command line, you are working in one of these directories. Commands you run can find files in that directory, but cannot (by default) find files in other directories.
> 
> When we say "change the working directory," we mean exactly that --- telling a terminal the directory in which you want to work.
> 
> To do this, we run the command
> 
> ```
> cd path\to\directory
> ```
> 
> where "`path\to\directory`" is replaced by the location of the directory in which you want to work. One easy way to find this location (i.e., the "path" to your directory) is by dragging its folder from the File Explorer into your terminal window (see documentation on [Quickly Copy Files Paths to Your Command Prompt via Drag and Drop](https://lifehacker.com/quickly-copy-file-paths-to-your-command-prompt-via-drag-5382503). In particular, I would first type "`cd `" (note the single trailing space):
> 
> ```
> C:\Users\jakek>cd 
> ```
> 
> Then, I would drag a folder into the terminal window and press enter. For instance, suppose I had created a folder called `ae353-sp25` somewhere on my computer and dragged it in, then pressed enter --- I would see something like this:
> 
> ```
> C:\Users\jakek>cd C:\Users\jakek\OneDrive\Documents\ae353-sp25
> C:\Users\jakek\OneDrive\Documents\ae353-sp25>
> ```
> 
> See documentation on [Find and Open Files using Windows Command Prompt](https://www.faqforge.com/windows/find-and-open-files-using-windows-command-prompt/) for a way to search for the directory location of files on your computer.

Use [git](https://git-scm.com/) to download the code from our [ae353 github repository]({{ site.github.repository_url }}) by running this command:

```
git clone https://github.com/tbretl/ae353-sp25.git
```

This process will take very little time. When it completes, you should find a new folder called `ae353-sp25` inside your working directory.

### Create a conda environment

An "environment" is like a sandbox where you can install software without causing any conflict with other things you might have installed on your computer. To create an environment for work in AE353, change your working director to `ae353-sp25` (wherever you put the code). Then, run this command:

```
conda env create -f environment.yml
```

You may see something like `WARNING: A conda environment already exists` and be asked if you want to `Remove existing environment`. This means that you already created an environment called `ae353`. Perhaps something went wrong and you are creating it again --- in this case, type `y` to remove the existing environment and proceed to recreate it. From then on, accept all default options.

Make sure that this process completes successfully. If you see any errors, ask for help.


### Install VS Code

[Visual Studio Code](https://code.visualstudio.com/) (aka VS Code) is the cross-platform code editor that we recommend for use with this course.

To install VS Code, follow the instructions for [Installation](https://code.visualstudio.com/docs/setup/windows#_installation) from the [Visual Studio Code on Windows](https://code.visualstudio.com/docs/setup/windows) setup page. 

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

You should see the prefix to your powershell prompt change from `(base)` to `(ae353)`. This means you are in the conda environment you created for work with AE353.

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

As an alternative to VS Code, if you like, you can also work with jupyter notebooks in a browser window. To do so, run this command:

```
jupyter notebook
```

A browser window should open with the jupyter notebook interface. You can now navigate to and open any of the notebooks (with extension `.ipynb`) used for in-class examples or for design projects.

In either case, **we strongly recommend you duplicate and work with a copy of any given notebook rather than working with the original.** Feel free to ignore this suggestion if you are a `git` expert.



