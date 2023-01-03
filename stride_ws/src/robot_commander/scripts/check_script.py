from robot_commander import RobotCommander

def has_method(o, name):
    return callable(getattr(o, name, None))


def check_script(script_name, brake = False):
    f = open(script_name, "r")
    name = None

    for line in f:
        stripped_line = line.strip()
        if line in ['\n', '\r\n', '']:
            continue

        if stripped_line.startswith('#'):
            continue

        if stripped_line.startswith('print'):
            continue

        if not name:
            a = line.split('=')

            if len(a) > 1 and a[1].strip() == 'RobotCommander()':
                name = a[0].strip()
                continue
            else:
                print('not rc')
                return False

        n = line.split('.')

        if len(n) > 1 and n[0].strip() == name:
            f, args = n[1].strip().split('(')
            if not has_method(RobotCommander, f):
                return False

            if (f == 'engage_brake_hill' or f == 'disengage_brake_hill') and not brake:
                print('Brake function does not support')
                return False
        else:
            return False

    return True


if __name__ == '__main__':
    print(check_script())



    

