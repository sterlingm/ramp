from colorama import init as clr_ama_init
from colorama import Fore
clr_ama_init(autoreset = True)

class RampRlLogger(object):
    """Log data in the format convenient for plotting.

    One line is one frame.
    The frequency of writing is controlled outside the class.
    Not too fast!
    """
    def __init__(self, file, names, sep_char=',', enable=True):
        """The initiation method of RampRlLogger class.

        Arguments
        ---------
            file: The full name of file to be used for logging.

            ames: Names of different tags. A list or np.array.

            enable: Intialize a RampRlLogger object with enabling it or not.
        """
        super(RampRlLogger, self).__init__()
        self.file_h = open(file, 'a')
        self.sep_char = sep_char
        self.enable = enable
        self.setColumnName(names)
        self.file_name = file



    def enable(self):
        self.enable = True



    def disable(self):
        self.enable = False



    def reOpen(self):
        if not self.file_h.closed:
            self.close()
        self.file_h = open(self.file_name, 'a')



    def open(self, file):
        if not self.file_h.closed:
            self.close()
        self.file_h = open(file, 'a')



    def close(self):
        if not self.file_h.closed:
            self.file_h.close()



    def setColumnName(self, names):
        """
        Arguments
        ---------
            names: Names of different tags. A list or np.array.
        """
        if self.file_h.closed:
            print(Fore.RED + "No file opened!")
            return False

        self.col_size = len(names)
        for i, name in enumerate(names):
            self.file_h.write(name)
            if i != len(names) - 1:
                self.file_h.write(self.sep_char)

        self.file_h.write('\n')
        return True



    def logOneFrame(self, values):
        """
        Arguments
        ---------
            values: Values of the data frame. A list or np.array.
        """
        if self.file_h.closed:
            print(Fore.RED + "No file opened!")
            return False

        if self.col_size != len(values):
            print(Fore.RED + "Length of data frame is wrong!")
            return False

        for i, value in enumerate(values):
            self.file_h.write(str(value))
            if i != len(values) - 1:
                self.file_h.write(self.sep_char)

        self.file_h.write('\n')
        return True