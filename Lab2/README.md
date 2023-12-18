For this lab, I was starting with the sample-project in the get-started file. 
For lab2.1, the first chanllenging part is to transfer the elf to binary file.
At first, I was just trying to start the openocd, but it didn't work. 
I asked the TA john, he told me I have to get the jtag rules and transfer the file into a bin file.
Also, I find out we need to use the sudo while open the udev and reloading the rules.
Also, there was permission needed for the port, I was just following one of the piazza post about this.

After, getting into the gdb part, at the beginning, I was literaly trying to gdb the elf file which should never work.
As the openocd works, I figure the just create a file named gdbinit, change the breakpoint to compute and 
run the command from the assignment document.
After I got the three values and the address info from the breakpoint, that's all for the part 1.

For lab2.2, I was starting with the code from Chatgpt and the only difficulity is to debugging it.
(p.s. I got pretty lucky, the code from chatgpt was pretty close to the right one, I just changged a few.)
