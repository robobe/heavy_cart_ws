from functools import lru_cache

class ValueMapper:
    def __init__(self, input_min, input_max, output_min, output_max, deadzone=0):
        """
        Initialize the ValueMapper.
        
        Args:
        - input_min (float): Minimum value of the input range.
        - input_max (float): Maximum value of the input range.
        - output_min (float): Minimum value of the output range.
        - output_max (float): Maximum value of the output range.
        - deadzone (float): Deadzone size (centered around 0, no output for inputs in this range).
        """
        self.input_min = input_min
        self.input_max = input_max
        self.output_min = output_min
        self.output_max = output_max
        self.deadzone = deadzone


    @lru_cache(maxsize=1)
    def mid_input_range(self):
        input_range = self.input_max - self.input_min
        mid = self.input_min + input_range/2
        return mid 
    
    def map_value(self, value):
        """
        Map the input value to the output range, considering the deadzone.
        
        Args:
        - value (float): Input value to be mapped.
        
        Returns:
        - float: Mapped output value.
        """
        # Handle deadzone: return 0 if the value is inside the deadzone
        

        # Normalize the input value within its range (excluding deadzone)
        input_range = self.input_max - self.input_min
        if abs(value-self.mid_input_range()) < self.deadzone:
            return 0
        
        output_range = self.output_max - self.output_min

        # Map the value, ignoring the deadzone
        normalized_value = (value - self.input_min) / input_range
        
        # Apply the mapping from input range to output range
        mapped_value = self.output_min + (normalized_value * output_range)

        # Clamp the mapped value within the output range
        mapped_value = max(self.output_min, min(mapped_value, self.output_max))

        return mapped_value


if __name__ == "__main__":
    mapper = ValueMapper(input_min=1000, input_max=2000, output_min=-2, output_max=2, deadzone=10)
    print(mapper.map_value(1508))
    print(mapper.mid_input_range.cache_info())
    print(mapper.map_value(1508))
    print(mapper.mid_input_range.cache_info())