\documentclass[11pt,a4paper]{article}

% ===============================
% Packages
% ===============================
\usepackage{amsmath, amssymb}
\usepackage{graphicx}
\usepackage{booktabs}
\usepackage{algorithm}
\usepackage{algorithmic}
\usepackage{geometry}
\usepackage{hyperref}
\usepackage{caption}
\usepackage{subcaption}
\usepackage{float}

\geometry{margin=1in}

% ===============================
% Title Information
% ===============================
\title{Unscented Kalman Filter Based Target Tracking \\
       with Radar and IR Sensor Fusion}
\author{Hyun Woo Cho}
\date{\today}

\begin{document}
\maketitle

\begin{abstract}
This report presents a simulation study of a target tracking system using an Unscented Kalman Filter (UKF) with joint fusion of radar and infrared (IR) sensor measurements. The target motion is modeled in Cartesian coordinates, while radar observations are provided in spherical coordinates. The effectiveness of the UKF in handling nonlinear measurement models is evaluated through numerical simulation.
\end{abstract}

\tableofcontents
\newpage

\section{Introduction}

Target tracking using heterogeneous sensors is a fundamental problem in signal processing and defense systems.
Radar sensors provide range and radial velocity information, while infrared (IR) sensors offer angular measurements with high accuracy.
Fusing these complementary modalities improves robustness and estimation accuracy.

The Unscented Kalman Filter (UKF) is particularly suitable for this problem due to the strong nonlinearities present in spherical-coordinate radar measurements.
This report focuses on:
\begin{itemize}
    \item Modeling radar and IR sensors with nonlinear observation functions
    \item Joint measurement update using UKF
    \item Performance evaluation through simulation
\end{itemize}

This work serves as a software-level validation prior to a planned FPGA-based hardware accelerator implementation.

\section{Problem Formulation}

\subsection{State Vector}

The target state is defined in Cartesian coordinates as:
\begin{equation}
\mathbf{x}_k =
\begin{bmatrix}
x & y & z & \dot{x} & \dot{y} & \dot{z}
\end{bmatrix}^T
\end{equation}

\subsection{Motion Model}

A constant-velocity model is assumed:
\begin{equation}
\mathbf{x}_{k+1} = f(\mathbf{x}_k) + \mathbf{w}_k
\end{equation}
where $\mathbf{w}_k \sim \mathcal{N}(0, Q)$ is process noise.

\subsection{Radar Measurement Model}

Radar observations are expressed in spherical coordinates:
\begin{equation}
\mathbf{z}^{\text{radar}} =
\begin{bmatrix}
r & \phi & \theta & \dot{r}
\end{bmatrix}^T
\end{equation}

The nonlinear observation function is:
\begin{equation}
\mathbf{z}^{\text{radar}} = h_{\text{radar}}(\mathbf{x}, \mathbf{x}_{ego}) + \mathbf{v}_{r}
\end{equation}

\subsection{IR Measurement Model}

The IR sensor provides angular measurements:
\begin{equation}
\mathbf{z}^{\text{IR}} =
\begin{bmatrix}
\phi & \theta
\end{bmatrix}^T
\end{equation}

\section{Unscented Kalman Filter}

\subsection{Sigma Point Generation}

The UKF approximates nonlinear transformations using deterministically chosen sigma points:
\begin{equation}
\chi_0 = \hat{\mathbf{x}}, \quad
\chi_i = \hat{\mathbf{x}} \pm \sqrt{(n+\lambda)P}
\end{equation}

\subsection{Prediction Step}

Each sigma point is propagated through the motion model:
\begin{equation}
\chi^{(i)}_{k+1|k} = f(\chi^{(i)}_k)
\end{equation}

\subsection{Joint Measurement Update}

Radar and IR measurements are concatenated into a joint observation vector:
\begin{equation}
\mathbf{z} =
\begin{bmatrix}
\mathbf{z}^{\text{radar}} \\
\mathbf{z}^{\text{IR}}
\end{bmatrix}
\end{equation}

\section{Unscented Kalman Filter}

\subsection{Sigma Point Generation}

The UKF approximates nonlinear transformations using deterministically chosen sigma points:
\begin{equation}
\chi_0 = \hat{\mathbf{x}}, \quad
\chi_i = \hat{\mathbf{x}} \pm \sqrt{(n+\lambda)P}
\end{equation}

\subsection{Prediction Step}

Each sigma point is propagated through the motion model:
\begin{equation}
\chi^{(i)}_{k+1|k} = f(\chi^{(i)}_k)
\end{equation}

\subsection{Joint Measurement Update}

Radar and IR measurements are concatenated into a joint observation vector:
\begin{equation}
\mathbf{z} =
\begin{bmatrix}
\mathbf{z}^{\text{radar}} \\
\mathbf{z}^{\text{IR}}
\end{bmatrix}
\end{equation}

Angular components are handled using circular statistics to avoid discontinuities at $\pm\pi$.

\section{Simulation Setup}

\subsection{Scenario Description}

The target follows a smooth three-dimensional trajectory with additive noise.
The ego platform is assumed stationary for simplicity.

\subsection{Noise Parameters}

\begin{itemize}
    \item Process noise covariance $Q$
    \item Radar measurement noise covariance $R_{\text{radar}}$
    \item IR measurement noise covariance $R_{\text{IR}}$
\end{itemize}

\subsection{Evaluation Metrics}

Tracking performance is evaluated using:
\begin{itemize}
    \item Position estimation error
    \item Trajectory visualization
\end{itemize}

\section{Results}

\begin{figure}[H]
    \centering
    \includegraphics[width=0.8\textwidth]{ukf_radar_ir.png}
    \caption{True target trajectory and UKF-estimated trajectory}
\end{figure}

The UKF successfully tracks the target despite nonlinear measurement models.
Joint fusion of radar and IR measurements improves stability compared to single-sensor updates.

\section{Discussion}

The simulation highlights several computational bottlenecks relevant to hardware implementation:
\begin{itemize}
    \item Sigma point generation requiring matrix square roots
    \item Nonlinear trigonometric functions (atan2, sin, cos)
    \item Covariance updates involving matrix multiplications
\end{itemize}

These observations motivate the development of FPGA accelerators using CORDIC and QR-decomposition-based square-root UKF architectures.

\section{Conclusion and Future Work}

This report demonstrated the feasibility of UKF-based radar–IR sensor fusion for target tracking.
Future work includes:
\begin{itemize}
    \item Square-root UKF implementation
    \item Fixed-point analysis
    \item FPGA-based accelerator design and synthesis
\end{itemize}

\begin{thebibliography}{9}

\bibitem{julier1997}
S. J. Julier and J. K. Uhlmann,
\textit{A New Extension of the Kalman Filter to Nonlinear Systems},
Proc. SPIE, 1997.

\bibitem{wan2000}
E. A. Wan and R. Van Der Merwe,
\textit{The Unscented Kalman Filter for Nonlinear Estimation},
IEEE Symposium, 2000.

\bibitem{maybeck}
P. S. Maybeck,
\textit{Stochastic Models, Estimation, and Control},
Academic Press.

\end{thebibliography}

\end{document}
