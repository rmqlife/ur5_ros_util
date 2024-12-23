import numpy as np
from scipy.optimize import linprog

# linear programming
def myLinearProg(M, v):
    """
    Decomposes the vector v as a linear combination of the columns of A
    with non-negative coefficients using linear programming.

    Parameters:
    M (np.ndarray): A 2D array where each column is a vector.
    v (np.ndarray): The target vector to decompose.

    Returns:
    np.ndarray: The coefficients of the linear combination (non-negative).
    """
    # Ensure that A is a 2D matrix and v is a 1D vector
    M = np.array(M)
    v = np.array(v)

    # Ensure that the number of rows in A matches the size of v
    if M.shape[0] != v.shape[0]:
        raise ValueError("Number of rows in A must match the number of elements in v.")

    # Objective function: We don't care about the objective, so we set it to zero
    c = np.zeros(M.shape[1])  # Coefficients for the objective function (arbitrary)

    # Constraints: A * [c1, c2, ..., cn] = v
    # Bounds: c1 >= 0, c2 >= 0, ..., cn >= 0
    bounds = [(0, None)] * M.shape[1]  # Non-negative coefficients

    # Solve the linear program using 'highs' method
    result = linprog(c, A_eq=M, b_eq=v, bounds=bounds, method='highs')

    if result.success:
        return result.x  # Return the coefficients
    else:
        raise ValueError("No solution found with non-negative coefficients.")

# Example usage
if __name__ == "__main__":
    M = np.array([[1, 0, 0], [0, 2, 0],  [0,0,-3], [0,0,3]])  # Matrix A with columns v1 and v2
    v = np.array([5, 10, -3])  # Target vector v

    try:
        coefficients = myLinearProg(M.T, v)
        print("Coefficients:", coefficients)
    except ValueError as e:
        print(e)
