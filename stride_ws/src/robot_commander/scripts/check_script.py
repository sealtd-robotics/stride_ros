from robot_commander import RobotCommander

def has_method(o, name):
    return callable(getattr(o, name, None))


def check_script(script_name, brake = False):
    with open(script_name, "r") as f:
        name = None
        return_text = "OK"

        for line in f:
            stripped_line = line.strip()
            if stripped_line in ['\n', '\r\n', '']: #ignore empty lines
                continue

            if stripped_line.startswith('#'): # ignore commented line
                continue

            if stripped_line.startswith('print'): #ignore print
                continue

            if stripped_line.startswith('target'): 
                continue

            if not name:
                a = stripped_line.split('=')

                if len(a) > 1 and a[1].strip() == 'RobotCommander()':
                    name = a[0].strip()
                    continue
                else:
                    return_text = "WARN: Did not find object initiated with RobotCommander() as first line. Test will be aborted."
                    return return_text

            n = stripped_line.split('.')

            if len(n) > 1 and n[0].strip() == name:
                f, args = n[1].strip().split('(')
                if not has_method(RobotCommander, f):
                    return_text = "WARN: Does not support command %s. Test will be aborted." % f
                    return return_text

                if (f == 'engage_brake_hill' or f == 'disengage_brake_hill') and not brake:
                    return_text = "WARN: Test script contains brake command. This is a non-brake platform. Test will be aborted."
                    return return_text
            else:
                return_text = "WARN: Incorrect format command \"%s\". Test will be aborted." % line
                return return_text

        return "OK"


if __name__ == '__main__':
    print(check_script())



    

