package frc.robot.util;

// A 2D Vector
public class Vector 
{
    private double x; // x mag
    private double y; // y mag

    // Creates a <1, 0> vector
    public Vector() 
    {
        this.x = 0;
        this.y = 0;
    }

    // Creates an <x, y> vector
    public Vector(double x, double y) 
    {
        this.x = x;
        this.y = y;
    }

    // Rounds a double to 2 decimal places
    private static double round(double val) 
    {
        return .01 * Math.round(val * 100); // Shift 2 digits over, round, move 2 back
    }

    // Adds two vectors
    public static Vector add(Vector v1, Vector v2) 
    {
        return new Vector(v1.x() + v2.x(), v1.y() + v2.y()); // Add the components
    }

    // Cross product
    public static double crossProduct(Vector v1, Vector v2)
    {
        return v1.getMag() * v2.getMag() * Math.cos(Math.toRadians(v1.getAngle() - v2.getAngle()));
    }

    // Returns the x component of the vector
    public double x() 
    {
        return this.x;
    }

    // Returns the y component of the vector
    public double y() 
    {
        return this.y;
    }

    // Return the polar angle of the vector
    public double getAngle() 
    {
        // Edge cases
        if (x == 0)
        {
            if (y >= 0)
                return 0;
            else
                return 180;
        }
        else if (y == 0)
        {
            if (x > 0)
                return 90;
            else
                return 270;
        }
        else if (x > 0 && y > 0) // First Quadrant
            return Math.toDegrees(Math.atan(Math.abs(x / y)));
        else if (x > 0 && y < 0) // Second Quadrant
            return Math.toDegrees(Math.atan(Math.abs(y / x))) + 90;
        else if (x < 0 && y > 0) // Third Quadrant
            return -Math.toDegrees(Math.atan(Math.abs(x / y)));
        else // Fourth Quadrant
            return -(Math.toDegrees(Math.atan(Math.abs(y / x))) + 90);
    }

    // Return the magnitude of the polar vector
    public double getMag() 
    {
        return Math.sqrt((this.x * this.x) + (this.y * this.y)); // Use pythagorean theorum to calculate magnitude
    }
    // Sets polar coordinates with the on [-180, 180], len in some units
    public void setPol(double len, double the) 
    {
        // If theta > 180 or theta < -180, move it back onto [-180, 180] domain
        while (the > 180)
        {
            the -= 360;
        }
        while (the < -180)
        {
            the += 360;
        }

        if (0 <= the && the < 90) // First Quadrant
        {
            double ref = the;
            y = Math.cos(Math.toRadians(ref));
            x = Math.sin(Math.toRadians(ref));
        }
        else if (90 < the && the <= 180) // Second Quadrant
        {
            double ref = the - 90;
            x = Math.cos(Math.toRadians(ref));
            y = -Math.sin(Math.toRadians(ref));
        }
        else if (the < 0) // Make a new vector in a positive x-quadrant, then flip the x axis
        {
            Vector v = new Vector();
            v.setPol(1, -the);
            x = -v.x();
            y = v.y();
        }

        x *= len; // Multiply by length
        y *= len;
    }

    // Set cartesian coordinates
    public void setCart(double x, double y) 
    {
        this.x = x;
        this.y = y;
    }

    // Turns the vector in the opposite direction
    public void negate() 
    {
        this.x = -this.x;
        this.y = -this.y;
    }

    // Given the vector input is a square, map the vector magnitude to an inscribed circle
    public void circlify()
    {
        if (x == 0 || y == 0)
            return;

        double ref = Math.atan(Math.abs(y) / Math.abs(x)); // In Radians

        if (Math.abs(x) > Math.abs(y))
            this.scale(Math.cos(ref));
        else
            this.scale(Math.sin(ref));
    }

    // Multiply by a scalar
    public void scale(double scalar) 
    {
        // multiply magnitude by maximum value
        setPol(this.getMag() * scalar, this.getAngle()); 
    }

    public Vector unit()
    {
        if (this.getMag() == 0)
            return new Vector();
        Vector v = new Vector();
        v.setPol(1, this.getAngle());
        return v;
    }

    // Return as a string (by default cartesian)
    public String toString() 
    {
        return this.toStringCart();
    }

    // Return as a string in cartesian coordinates
    public String toStringCart() 
    {
        return "Vector X: " + round(this.x) + " Y: " + round(this.y);
    }

    // Return as a string in polar coordinates
    public String toStringPol() 
    {
        return "Vector Mag: " + round(this.getMag()) + " @ Pos: " + round(this.getAngle());
    }
}