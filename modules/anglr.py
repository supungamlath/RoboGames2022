# Planar angle mathematics module for Python
# Modified from Anthony Zhang's anglr library

import math, numbers

TWO_PI = math.pi * 2
DIRECTION_TO_ANGLE = {"N": 0, "E": math.pi/2, "S": math.pi, "W": 3*math.pi/2}


class Angle:
    def __init__(self, value, unit = "radians"):
        """
        Creates an `Angle` instance representing the angle `value` in the angular unit specified by `unit`.
        
        `unit` can be "radians", "degrees", "gradians", "hours", "arcminutes", "arcseconds", or "vector" (see `angle_instance.vector` for more info).
        """
        self.radians = 0

        if isinstance(value, Angle): self.radians = value.radians
        elif unit == "vector": self.vector(value)
        elif not isinstance(value, numbers.Real): raise ValueError(f"Value \"{value}\" must be a real-number-like object")
        elif unit == "radians": self.radians = value
        elif unit == "degrees": self.degrees(value)
        elif unit == "gradians": self.gradians(value)
        elif unit == "hours": self.hours(value)
        elif unit == "arcminutes": self.arcminutes(value)
        elif unit == "arcseconds": self.arcseconds(value)
        else: raise ValueError(f"Angle mode \"{unit}\" must be one of \"radians\", \"degrees\", \"gradians\", \"hours\", \"arcminutes\", \"arcseconds\", or \"vector\"")
        self.normalize()

    # various conversions between angles and numerical representations of angles
    def degrees(self, value):
        """Sets the represented angle to the angle represented by `value` in degrees."""
        self.radians = math.radians(value)

    def gradians(self, value):
        """Sets the represented angle to the angle represented by `value` in gradians."""
        self.radians = value * TWO_PI / 400

    def hours(self, value):
        """Sets the represented angle to the angle represented by `value` in hours."""
        self.radians = value * TWO_PI / 24

    def arcminutes(self, value):
        """Sets the represented angle to the angle represented by `value` in arcminutes."""
        self.radians = (value / 60) * TWO_PI / 360

    def arcseconds(self, value):
        """Sets the represented angle to the angle represented by `value` in arcseconds."""
        self.radians = (value / 3600) * TWO_PI / 360

    def vector(self, value):
        """Sets the represented angle to the angle that `value`, a vector `(X_VALUE, Y_VALUE)`, has counterclockwise from the positive X axis (standard position)."""
        if len(value) != 2: raise ValueError(f"Invalid vector \"{value}\"")
        if value[0] == 0 == value[1]: raise ValueError("Zero vector (0, 0) has undefined angle")
        self.radians = math.atan2(value[1], value[0])
    
    # make it behave like a real number
    def __add__(self, angle):
        if not isinstance(angle, Angle): raise ValueError(f"Addend \"{angle}\" must be an angle")
        return Angle(self.radians + angle.radians)
    def __sub__(self, angle):
        if not isinstance(angle, Angle): raise ValueError(f"Subtrahend \"{angle}\" must be an angle")
        return Angle(self.radians - angle.radians)
    def __mul__(self, value):
        if not isinstance(value, numbers.Real): raise ValueError(f"Multiplicand \"{value}\" must be numerical")
        return Angle(self.radians * value)
    def __truediv__(self, value):
        if not isinstance(value, numbers.Real): raise ValueError(f"Divisor \"{value}\" must be numerical")
        return Angle(self.radians / value)
    def __rmul__(self, value):
        if not isinstance(value, numbers.Real): raise ValueError(f"Multiplicand \"{value}\" must be numerical")
        return Angle(value * self.radians)
    def __neg__(self): return Angle(TWO_PI-self.radians)
    def __pos__(self): return Angle(self.radians)
    def __abs__(self): return Angle(abs(self.radians))
    def __round__(self): return Angle(round(self.radians))
    def __lt__(self, angle):
        if isinstance(angle, Angle): return self.radians < angle.radians
        return NotImplemented
    def __le__(self, angle):
        if isinstance(angle, Angle): return self.radians <= angle.radians
        return NotImplemented
    def __eq__(self, angle):
        if isinstance(angle, Angle): return self.radians == angle.radians
        return NotImplemented
    
    # type conversions
    def __complex__(self): return complex(self.radians)
    def __int__(self): return int(self.radians)
    def __float__(self): return float(self.radians)
    def __str__(self): return f"{self.radians} rad"
    def __repr__(self): return f"<Angle {self.radians} rad>"
    def __hash__(self): return hash(self.radians)
    
    # unit circle functions
    def normalize(self):
        """Returns a new `Angle` instance that represents the angle normalized on the unit circle to be between `lower` (inclusive) and `upper` (exclusive, defaults to `lower + TAU`)."""
        self.radians = self.radians % TWO_PI

    def angleBetweenCW(self, angle):
        """Returns a new `Angle` instance that represents the clockwise angle from this `Angle` instance to `angle` on the unit circle (this is always non-negative)."""
        return Angle((angle.radians - self.radians) % TWO_PI)

    def angleBetweenCCW(self, angle):
        """Returns a new `Angle` instance that represents the counter clockwise angle from this `Angle` instance to `angle` on the unit circle (this is always non-negative)."""
        return angle.angleBetweenCW(self)

    def signedValueBetween(self, angle):
        angle_cw = self.angleBetweenCW(angle)
        angle_ccw = self.angleBetweenCCW(angle)
        if angle_cw < angle_ccw: 
            return angle_cw.radians
        return -angle_ccw.radians

    def getSignedError(self, orientation):
        return self.signedValueBetween(Angle(DIRECTION_TO_ANGLE[orientation]))

    def angleBetween(self, angle):
        """Returns a new `Angle` instance that represents the smallest of the two possible angles between `Angle` instance to `angle` on the unit circle (this is always non-negative)."""
        return min(self.angleBetweenCW(angle), self.angleBetweenCCW(angle))

    def angleTo(self, angle):
        """Returns a new `Angle` instance that represents the angle with the smallest magnitude that, when added to this `Angle` instance, results in `angle` on the unit circle."""
        clockwise = self.angleBetweenCW(angle)
        counterclockwise = -angle.angleBetweenCW(self)
        return clockwise if clockwise <= abs(counterclockwise) else counterclockwise

    @staticmethod
    def getIdealAngleCW(orientation):
        return Angle(DIRECTION_TO_ANGLE[orientation] + math.pi / 2)

    @staticmethod
    def getIdealAngleCCW(orientation):
        return Angle(DIRECTION_TO_ANGLE[orientation] - math.pi / 2)