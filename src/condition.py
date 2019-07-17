CONDITION_F    = 0
CONDITION_F_CT = 1
CONDITION_T    = 2
CONDITION_T_CT = 3

# eval_fun: a function that when called evaluates the condition and returns True or False
# on_cycles: the number of cycles that the condition must be true for
# off_cycles: the number of cycles that the condition must be false for
class Condition:
    def __init__(self, eval_fun, on_cycles, off_cycles, start):
        self.eval_fun = eval_fun
        self.on_cycles = on_cycles
        self.off_cycles = off_cycles
        self.state = CONDITION_T if start else CONDITION_F
        self.counter = self.off_cycles if start else self.on_cycles
    def __call__(self):
        value = self.eval_fun()
        if self.state == CONDITION_F:
            self.counter = self.on_cycles
            self.state = CONDITION_F_CT if value else CONDITION_F
        elif self.state == CONDITION_F_CT:
            self.counter = self.counter - 1
            if self.counter:
                self.state = CONDITION_F_CT if value else CONDITION_F
            else:
                self.state = CONDITION_T
        elif self.state == CONDITION_T:
            self.counter = self.off_cycles
            self.state = CONDITION_T if value else CONDITION_T_CT
        else:
            self.counter = self.counter - 1
            if self.counter:
                self.state = CONDITION_T if value else CONDITION_T_CT
            else:
                self.state = CONDITION_F
        return self.state == CONDITION_T or self.state == CONDITION_T_CT
    def get_value(self):
        return self.state == CONDITION_T or self.state == CONDITION_T_CT
    def reset(self, start):
        self.state = CONDITION_T if start else CONDITION_F
        self.counter = self.off_cycles if start else self.on_cycles
