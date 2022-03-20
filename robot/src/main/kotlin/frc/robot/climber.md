### testing items
- determine that limit switches actually work
  - detach motor from winch assembly
  - use the voltage-based system
  - check that motor stops moving whenever one of the switches is bumped
- determine that encoder zeroes / maxes when it ought to
  - under the same conditions as the above test, check that the value is correct
  - with motor detached, let it spin (giving encoder value), hit bottom switch/top switch
- make sure pid plays well with limit switches
  - switch default command to the pid system, put the setpoint at something vastly higher
  - press both of the switches: if they stop the motor we're good
  - if it doesn't, add a term to turn the pid setpoint to zero?
- determine units of encoder
  - if not inches, convert to inches (not strictly necessary)
- find max encoder value assuming that bottom is zero
  - use the pid command


- test breakpoints of appropriate heights
- input values into the appropriate commands (write the code)


- record configs of all motors (download a file of them)
- ask edward about using bang-bangs