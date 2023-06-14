class Controller:
    def __init__(self, mapping: dict=None):
        self._inputs = dict()
        if mapping:
            for input, params in mapping.items():
                if params['type'] == 'button':
                    self._inputs[input] = Button(params)
                elif params['type'] == 'axis':
                    self._inputs[input] = Axis(params)

    def update_states(self, axes, buttons):
        for _, input in self._inputs.items():
            input_type = type(input)
            if input_type == Axis:
                input.update(axes)
            elif input_type == Button:
                input.update(buttons)

    def __getitem__(self, item):
        return self._inputs[item]

    def __iter__(self):
        return self._inputs.__iter__()

    def __str__(self):
        return '{' + ', '.join(['%s.: %s.' % (key, value.state) for key, value in self._inputs.items()]) + '}'

    def __len__(self):
        return len(self._inputs)


class Axis:
    def __init__(self, params):
        self._supported_params = ('index', 'type', 'exclusion', 'scale', 'offset')
        self._specified_params = params.keys()

        for param in self._specified_params:
            if param not in self._supported_params:
                raise ValueError('Type \"%s.\" is not supported' % param)

        if 'index' in self._specified_params:
            self._index = params['index']
        else:
            raise ValueError('Parameter "index" required.')

        self._exclusion = params['exclusion'] if 'exclusion' in self._specified_params else None
        self._scale = params['scale'] if 'scale' in self._specified_params else 1
        self._offset = params['offset'] if 'offset' in self._specified_params else 0
        self.state = 0

    def update(self, states):
        state = states[self._index]
        if self._exclusion and min(self._exclusion) <= state <= max(self._exclusion):
            state = 0

        self.state = self._scale * state + self._offset

class Button:
    def __init__(self, params):
        self._supported_params = ('index', 'type', 'boolean', 'latch')
        self._specified_params = params.keys()

        for param in self._specified_params:
            if param not in self._supported_params:
                raise ValueError('Type \"%s.\" is not supported' % param)

        if 'index' in self._specified_params:
            self._index = params['index']
        else:
            raise ValueError('Parameter "index" required.')

        self._is_bool = 'boolean' in self._specified_params
        self._is_latch = 'latch' in self._specified_params

        self._prev_press = 0
        if self._is_bool:
            self.state = False
        else:
            self.state = 0
        
            
    def update(self, states):
        state = states[self._index]
        if self._is_latch:
            if self._prev_press == 0 and state == 1:
                self.state = not self.state if self._is_bool else int(not self.state)
        else:
            self.state = bool(state) if self._is_bool else state
        self._prev_press = state

     
        

