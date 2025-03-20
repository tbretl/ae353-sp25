In order to use the examples in this directory, you will need to create a new conda environment. Change your working directory to the one containing this `README.md` file (to `ae353-sp25/examples/20250305`) and run the following command:

```
conda env create -f environment.yml
```

Then, activate this environment:

```
conda activate ae353-bokeh
```

Then, run the examples as follows:

```
bokeh serve error-vs-effort.py
```

or

```
bokeh server lqr-wheel.py
```