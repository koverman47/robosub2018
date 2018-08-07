#!/usr/bin/env python

import math


def add_vector(a, b):
    c = []
    for i in range(len(a)):
        c.append(a[i] + b[i])
    return c


def add_matrix(a, b):
    c = []
    for i in range(len(a)):
        c.append(add_vector(a[i], b[i]))
    return c


def subtract_vector(a, b):
    c = []
    for i in range(len(a)):
        c.append( a[i] - b[i] )
    return c


def subtract_matrix(a, b):
    c = []
    for i in range(len(a)):
        c.append(subtract_vector(a[i], b[i]))
    return c


def inner_product(a, b):
    return sum(a[i] * b[i] for i in range(len(a)))

def outer_product(a, b):
    outer = []
    for i in range(len(a)):
        outer.append([])
        for j in range(len(b)):
            outer.[i].append(a[i] * b[j])
    return outer


def convolution(a, b):
    pass


def cross_product(a, b):
    pass


# Naive Matrix multiplication
def matrix_multiplication(a, b):
    c = []
    for x in range(len(a)):
        c_row = []
        for y in range(len(a[x])):
            sigma = 0
            for z in range(len(b)):
                sigma += a[x][y] * b[y][z]
            c_row.append(sigma)
        c.append(c_row)
                


def constant_multiply_vector(a, k):
    c = []
    for i in range(len(a)):
        #c.append(round(a[i] * k, 8))
        c.append(a[i] * k)
    return c


def constant_multiply_matrix(a, k):
    c = []
    for v in a:
        c.append(constant_multiply_vector(v, k))
    return c


def invert(a, b):
    pass

def recursive_vector_sum(vector, n):
    if n == 1:
        return vector[-1]
    return add_vector(vector[-n], recursive_vector_sum(vector, n - 1))




