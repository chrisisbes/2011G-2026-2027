#ifndef GLOBALS_HPP
#define GLOBALS_HPP

// Global variables
inline bool intake_active = false;                  // Flag for enabling/disabling the intake task
inline bool autonomous_mode = false;              // Flag for autonomous mode
inline bool reverse_on_red = false;  
inline bool is_jam_detected = false;
inline unsigned long jam_timer = 0;
inline unsigned long grace_timer = 0;
inline bool task_mode = true;

#endif