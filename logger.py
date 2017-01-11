import rospy

class Logger:
    def __init__(self, name):
        self.name = "[" + str(name) + "]: "
    
    def _print(self, printer, msg):
        printer(self.name + str(msg))
    
    def debug(self, msg, var_name = None, method = None, line = None):
        if var_name is not None:
            msg = var_name + " = " + str(msg)
            
        if method is not None:
            msg = "In method '" + method + "': " + str(msg)
            
        if line is not None:
            msg = "(line " + str(line) + ")"
            
        self._print(rospy.loginfo, msg)
        
    def error(self, msg, method_name = None):
        if method_name is not None:
            msg = "Error in method '" + method_name + "': " + str(msg)
            
        self._print(rospy.logerr, msg)
    
    def info(self, msg):
        self._print(rospy.loginfo, msg)
    
    def warn(self, msg):
        self._print(rospy.logwarn, msg)