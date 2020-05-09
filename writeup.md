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

`\begin{align*}
\tau_x &= (F_1-F_2-F_3+F_4)l \\
\tau_y &= (F_1+F_2-F_3-F_4)l \\
\tau_z &= \tau_1+\tau_2+\tau_3+\tau_4 \\
F_c &= F_1 + F_2 + F_3 + F_4 \\
\end{align*}`