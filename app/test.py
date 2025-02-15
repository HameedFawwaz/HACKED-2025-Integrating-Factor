import nav

c = nav.Data()

# Generate test cases
test_cases = [
    (0, 0, 1, 1, 2, 2, 3, 3, 3, 0.1, 0.2, 0.3, 5, 5, 5),
    (1, 1, 1, 1, 1, 1, 2, 2, 2, 0.5, 0.5, 0.5, 10, 10, 10),
    (2, 3, 4, 0, 0, 0, 5, 5, 5, 1.2, 1.5, 1.8, 15, 12, 8),
    (-1, -2, -3, 3, 2, 1, 6, 6, 6, -0.2, -0.4, -0.6, -5, -3, -2),
    (4, 3, 2, 1, 1, 1, 8, 9, 7, 2.5, 3.1, 1.8, 20, 18, 15),
]

# Call update_data with each test case
for case in test_cases:
    c.update_data(*case)
    

print(c.data)