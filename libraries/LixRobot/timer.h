/**
 * @file timer.h
 * @brief Helper class that maintains a timer.
 * @author M. Syamlal
 */
class Timer
{
public:
    /*
     * @brief Class constructor.
     * @param Timer interval.
     */
    Timer (unsigned int i)
    {
        interval = i;
        start();
    }
    
    /*
     * @brief set timer interval.
     * @param Timer interval.
     */
    void set(unsigned int i)
    {
        interval = i;
    }
    
    /*
     * @brief start the timer.
     */
    void start()
    {
        startTime = millis();
    }
    
    /*
     * @brief check whether the timer is done.
     * @return true if the timer is done.
     */
    bool done()
    {
        if((millis() - startTime) > interval){
            startTime = millis();
            return true;
        }
        else{            
            return false;
        }
    }
    
private:
    unsigned int interval;
    unsigned long startTime;
};
