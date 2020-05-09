<script src="//yihui.org/js/math-code.js"></script>
<!-- Just one possible MathJax CDN below. You may use others. -->
<script async
  src="//mathjax.rstudio.com/latest/MathJax.js?config=TeX-MML-AM_CHTML">
</script>

# Writeup - Project 3, Building a Controller
## Udacity Flying Car Nanodegree

## Overview
The project involved implementing a cascaded flight controller for a quadcopter drone.

<image src="assets/cascaded_controller.png" height="250">  

*[Image from [Udacity](www.udacity.com)]*

## Imlementation
### Generate motor commands
Tasks to implement the `QuadControl::GenerateMotorCommands` class method in `QuadCOntrol.cpp`.

<img src="https://latex.codecogs.com/gif.latex?\begin{align*}&space;\tau_x&space;&=&space;(F_1-F_2-F_3&plus;F_4)l&space;\\&space;\tau_y&space;&=&space;(F_1&plus;F_2-F_3-F_4)l&space;\\&space;\tau_z&space;&=&space;\tau_1&plus;\tau_2&plus;\tau_3&plus;\tau_4&space;\\&space;F_c&space;&=&space;F_1&space;&plus;&space;F_2&space;&plus;&space;F_3&space;&plus;&space;F_4&space;\\&space;\end{align*}" title="\begin{align*} \tau_x &= (F_1-F_2-F_3+F_4)l \\ \tau_y &= (F_1+F_2-F_3-F_4)l \\ \tau_z &= \tau_1+\tau_2+\tau_3+\tau_4 \\ F_c &= F_1 + F_2 + F_3 + F_4 \\ \end{align*}" />