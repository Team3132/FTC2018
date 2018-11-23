package org.firstinspires.ftc.teamcode.Auto;

public abstract class AutoCommand {
    protected boolean running;

    public AutoCommand() {
        this.running = true;
    }

    /**
     * Init
     *
     * This is run once at the start of the command. Can be used to set things up.
     */
    public abstract void init();


    /**
     * Periodic
     *
     * This is run periodically during the command.
     */
    public abstract void periodic();


    public boolean isRunning() {
        return this.running;
    }
}
