# using Pkg; Pkg.add("ControlSystems"); Pkg.add("Plots"); # comment out if these packages are already added
using Plots # for plotting outputs
using ControlSystems # control sys
using LinearAlgebra  # for identity matrix I

function get_closed_loop(k=20.2, T=1/20.2, ωp=20.1, phasemargin=60.1)
    # create a generatic PI controller that uses the open-loop response of a transfer function to
    # calculate the ideal frequency response for a first order transfer function response

    P = tf(1,[T,1]); # create a representation of the first order transfer function

    kp, ki, C = loopshapingPI(P,ωp,phasemargin=phasemargin);

    P_cl = feedback(tf(P)*tf(C)); # build closed-loop system

    return P_cl, P, kp, ki

end

P_cl, P, kp, ki = get_closed_loop();

print("Closed-Loop Proportional Gain Kp = "); println(kp);
print("Closed-Loop Integral Gain Ki = "); println(ki);

step_plot = plot!(step(P_cl))
nyquist_plot = plot(nyquistplot([P, P_cl], ylims=(-1,1), xlims=(-1.5,1.5)))
savefig(step_plot,"./julia/figs/step.png")
savefig(nyquist_plot,"./julia/figs/nyquist.png")