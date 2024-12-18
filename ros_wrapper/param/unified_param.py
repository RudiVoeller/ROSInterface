class UnifiedParameter:
    """
      A class to represent a unified parameter.

      Attributes:
          value: The value of the parameter.
      """
    def __init__(self, value):
        """
                Initializes the UnifiedParameter with the given value.

                Args:
                    value: The value of the parameter.
                """
        self.__value = value

    def get_value(self):
        """
                Gets the value of the parameter.

                Returns:
                    The value of the parameter.
                """
        return self.__value