class MonolithicControlEntity;

/**
 * @brief Abstract base class for all flight states.
 *
 * This defines the interface that all concrete states must implement.
 */
class State
{
public:
    virtual ~State() = default;

    // Called once when the state is entered.
    virtual void on_enter(MonolithicControlEntity *mce) { (void)mce; }

    // Called continuously while the state is active.
    virtual State *on_run(MonolithicControlEntity *mce) = 0;

    // Called once when the state is exited.
    virtual void on_exit(MonolithicControlEntity *mce) { (void)mce; }

    virtual const char *getName() const = 0;
};


class DisarmedState : public State
{
    // Motors should not be running. Should stay in this state until throttle is at 0 and the arm switch is engaged
public:
    static State *instance();
    virtual void on_enter(MonolithicControlEntity *mce) override;
    virtual State *on_run(MonolithicControlEntity *mce) override;
    virtual const char *getName() const override { return "DISARMED"; }
};

class FailsafeState : public State
{
    // Transitioned to when we lose positive control for set time
    // Right now, let's just cut motors and yeet to the ground, but could do a return home in the future.
public:
    static State *instance();
    virtual void on_enter(MonolithicControlEntity *mce) override;
    virtual State *on_run(MonolithicControlEntity *mce) override;
    virtual const char *getName() const override { return "FAILSAFE"; }
};

class ArmedState : public State
{
    // Runs the auto level controller
public:
    static State *instance();
    virtual void on_enter(MonolithicControlEntity *mce) override;
    virtual State *on_run(MonolithicControlEntity *mce) override;
    virtual const char *getName() const override { return "ARMED"; }
};
