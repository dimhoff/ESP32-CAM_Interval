# TODO's
 - Switch to Seconds for interval configuration in config file and set-up mode
 - Add configuration option to disable booting into set-up mode
 - Try optimizing power consumption during active time
 - OV5640 camera support

# Investigate
 - Cache configuration object(cfg) in RTC memory to improve wake-up time
 - Add option to take Multi/burst shot when triggered instead of one. This
   might be usefull if the interval between shots is very long. In that case it
   is rather anoying if there is i.e. a fly on the lens, right at the moment
   the camera triggers.

# Known Issues
 - Currently configured timezone location is not available in set-up mode
   configuration page. Because multiple locations have the same TZ string, it
   is impossible to determine the previously selected option based on the
   configuration.
