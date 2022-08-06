# using Pkg; Pkg.add("ControlSystems"); Pkg.add("Plots"); # comment out if these packages are already added
using Plots # for plotting outputs
using ControlSystems # control sys
using LinearAlgebra  # for identity matrix I

function get_closed_loop(k=20.2, T=1/20.2, ωp=20.1, phasemargin=60.1)
    # create a generatic PI controller that uses the open-loop response of a transfer function to
    # calculate the ideal frequency response

    P = tf(1,[T,1]); # create a representation of the first order transfer function

    C, kp, ki = loopshapingPI(P,ωp,phasemargin=phasemargin);

    P_cl = feedback(tf(P)*tf(C)); # build closed-loop system

    return P_cl, P, kp, ki

end

P_cl, P = get_closed_loop();

print("Closed-Loop Transfer Function")
print(P_cl);

step_plot = plot!(step(P_cl))
nyquist_plot = plot(nyquistplot([P, P_cl], ylims=(-1,1), xlims=(-1.5,1.5)))
savefig(step_plot,"./julia/figs/step.png")
savefig(nyquist_plot,"./julia/figs/nyquist.png")






# ζ = 0.5 # Desired damping

# ws = logspace(-1,2,8) # A vector of closed-loop bandwidths
# kp = 2*ζ*ws-1 # Simple pole placement with PI given the closed-loop bandwidth, the poles are placed in a butterworth pattern
# ki = ws.^2
# pidplots(P,:nyquist,:gof;kps=kp,kis=ki, ω= logspace(-2,2,500)) # Request Nyquist and Gang-of-four plots (more plots are available, see ?pidplots )
