# -*- coding: utf-8 -*-
"""
Created on Sun Feb 22 16:06:36 2026

@author: gunni
"""

import numpy as np

def check_stability_constraints(kx, kv, kR, kOmega, c1, c2, alpha=0.9, B=1.0, ev_max=1.0):
    """
    Checks if the current gain set satisfies the almost-global exponential 
    stability criteria for the Position Controlled Flight Mode.
    Assumes an abstracted plant where m = 1 and J = Identity Matrix.
    """
    results = {
        "is_safe": True,
        "failed_checks": []
    }
    
    # 1. Check bounds for c1
    c1_limit_1 = kv * (1 - alpha)
    c1_limit_2 = (4 * kx * kv * (1 - alpha)) / ((kv**2) * (1 + alpha)**2 + 4 * kx)
    c1_limit_3 = np.sqrt(kx)
    c1_max = min(c1_limit_1, c1_limit_2, c1_limit_3)
    
    if c1 >= c1_max:
        results["is_safe"] = False
        results["failed_checks"].append(f"c1 ({c1}) exceeds maximum allowed ({c1_max:.4f})")

    # 2. Check bounds for c2
    c2_limit_1 = kOmega
    c2_limit_2 = (4 * kOmega * kR) / (kOmega**2 + 4 * kR)
    c2_limit_3 = np.sqrt(kR)
    c2_max = min(c2_limit_1, c2_limit_2, c2_limit_3)
    
    if c2 >= c2_max:
        results["is_safe"] = False
        results["failed_checks"].append(f"c2 ({c2}) exceeds maximum allowed ({c2_max:.4f})")

    # 3. Construct the W matrices to check eigenvalue constraints
    # W1 matrix definition
    W1 = np.array([
        [c1 * kx, -0.5 * c1 * kv * (1 + alpha)],
        [-0.5 * c1 * kv * (1 + alpha), kv * (1 - alpha) - c1]
    ])
    
    # W12 matrix definition
    W12 = np.array([
        [kx * ev_max + c1 * B, 0],
        [B, 0]
    ])
    
    # W2 matrix definition
    W2 = np.array([
        [c2 * kR, -0.5 * c2 * kOmega],
        [-0.5 * c2 * kOmega, kOmega - c2]
    ])
    
    # 4. Check the final eigenvalue ratio constraint
    eigvals_W1 = np.linalg.eigvals(W1)
    eigvals_W2 = np.linalg.eigvals(W2)
    
    # Verify matrices are positive-definite (eigenvalues > 0)
    if np.any(eigvals_W1 <= 0) or np.any(eigvals_W2 <= 0):
        results["is_safe"] = False
        results["failed_checks"].append("W1 or W2 is not positive-definite.")
    else:
        lambda_min_W1 = np.min(eigvals_W1)
        lambda_min_W2 = np.min(eigvals_W2)
        norm_W12 = np.linalg.norm(W12, ord=2)
        
        ratio_threshold = (4 * norm_W12**2) / lambda_min_W1
        
        if lambda_min_W2 <= ratio_threshold:
            results["is_safe"] = False
            results["failed_checks"].append(
                f"Eigenvalue constraint failed: lambda_min(W2)={lambda_min_W2:.4f} "
                f"must be > {ratio_threshold:.4f}"
            )

    return results

# --- Example Usage ---
# Safe gains (arbitrary example)
status = check_stability_constraints(kx=1, kv=1, kR=10, kOmega=5, c1=0.01, c2=.2,B=471)
print(f"Are gains safe? {status['is_safe']}")
if not status["is_safe"]:
    for failure in status["failed_checks"]:
        print(f" - {failure}")