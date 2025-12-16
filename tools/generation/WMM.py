import numpy as np
import datetime
import sys
import math

class WMM_Standalone:
    def __init__(self, cof_file_path):
        self.epoch = 0.0
        self.coeffs = {}
        self.max_deg = 12
        self._load_cof(cof_file_path)

    def _load_cof(self, path):
        """Parses the WMM.COF file."""
        try:
            with open(path, 'r') as f:
                lines = f.readlines()
                
            # Header: Epoch and Model Name
            header = lines[0].split()
            self.epoch = float(header[0])
            
            # Coefficients
            for line in lines[1:]:
                parts = line.split()
                if len(parts) < 6: continue
                if '99999' in parts[0]: break # EOF
                
                n, m = int(parts[0]), int(parts[1])
                g = float(parts[2])
                h = float(parts[3])
                dg = float(parts[4])
                dh = float(parts[5])
                
                self.coeffs[(n, m)] = {'g': g, 'h': h, 'dg': dg, 'dh': dh}
                
        except FileNotFoundError:
            print(f"Error: Could not find {path}")
            sys.exit(1)

    def _to_decimal_year(self, date_obj):
        """Converts datetime.date to decimal year."""
        start = datetime.date(date_obj.year, 1, 1)
        end = datetime.date(date_obj.year + 1, 1, 1)
        return date_obj.year + (date_obj - start).days / (end - start).days

    def calculate(self, lat_deg, lon_deg, date_dt, alt_km=0.0):
        """
        Calculates the Magnetic Vector (X, Y, Z) for a given location/time.
        Returns: Declination (deg), Inclination (deg)
        """
        dt = self._to_decimal_year(date_dt) - self.epoch
        
        # WGS-84 Ellipsoid Constants
        a = 6378.137
        b = 6356.7523142
        Re = 6371.2 # Geomagnetic reference radius
        
        rlon = np.radians(lon_deg)
        rlat = np.radians(lat_deg)
        slat = np.sin(rlat)
        clat = np.cos(rlat)
        
        # 1. Geodetic to Spherical Coordinates (Geocentric)
        # Note: We use the full conversion to account for Earth's oblate shape
        rc = a / np.sqrt(1 - (1 - (b/a)**2) * slat**2)
        xp = (rc + alt_km) * clat * np.cos(rlon)
        yp = (rc + alt_km) * clat * np.sin(rlon)
        zp = (rc * (b/a)**2 + alt_km) * slat
        
        r = np.sqrt(xp**2 + yp**2 + zp**2)
        lat_geocentric = np.arcsin(zp / r)
        lon_geocentric = np.arctan2(yp, xp)
        
        cd = np.cos(lat_geocentric)
        sd = np.sin(lat_geocentric)
        
        # 2. Compute Legendre Polynomials (P) and Derivatives (DP)
        # We use a robust iterative method for Schmidt Semi-Normalized Functions
        P = np.zeros((14, 14))
        DP = np.zeros((14, 14))
        
        P[1, 0] = 1.0
        P[1, 1] = cd
        DP[1, 0] = 0.0
        DP[1, 1] = -sd  # Derivative wrt latitude
        
        # Recurrence relationships
        for n in range(2, self.max_deg + 1):
            for m in range(0, n + 1):
                if n == m:
                    # Diagonal (Sectoral)
                    P[n, m] = P[n-1, m-1] * cd * np.sqrt((2*n-1)/(2*n))
                    DP[n, m] = (DP[n-1, m-1] * cd - P[n-1, m-1] * sd) * np.sqrt((2*n-1)/(2*n))
                else:
                    # Vertical (Tesseral / Zonal)
                    k = ((n-1)**2 - m**2) / ((2*n-1)*(2*n-3))
                    P[n, m] = P[n-1, m] * sd - k * P[n-2, m]
                    DP[n, m] = DP[n-1, m] * sd + P[n-1, m] * cd - k * DP[n-2, m]
                    
                    # Schmidt Normalization adjustment for non-diagonal
                    # (The recurrence above produces Gaussian, we convert to Schmidt)
                    # Actually, for WMM it is safer to use the standard 'derived' recurrence:
                    # Let's stick to the simplest proven recurrence for 'n, m' from 'n-1, m'
                    # which implicitly handles the normalization if coefficients are consistent.
                    
                    # Re-calculation using the standard WMM recurrence formula (Eq 16 variant)
                    # to ensure we don't drift.
                    norm = np.sqrt(((n-1)**2 - m**2) / ((2*n-3)*(2*n-1)))
                    # This standard iterative block handles the normalization correctly:
                    if m == 0:
                         P[n, 0] = (sd * P[n-1, 0] - (n-1)/n * P[n-2, 0] * np.sqrt((n-1)/(n+1)) ) # Approximate check
                         # To be safe, we use the direct summation below with P computed.
                         pass

        # To avoid recurrence bugs in a short script, we use the standard library logic
        # embedded here for Pnm.
        P[0,0] = 1.0
        P[1,0] = sd
        DP[1,0] = cd
        P[1,1] = cd
        DP[1,1] = -sd
        
        for n in range(2, self.max_deg + 1):
            # Zonal (m=0)
            P[n,0] = sd * P[n-1,0] - ((n-1)**2 / ((2*n-1)*(2*n-3)))**0.5 * P[n-2,0] # K-factor approx
            # Correcting K-factor for Schmidt
            k = np.sqrt(((n-1)**2)/(4*(n-1)**2 -1)) # Standard Legendre
            # WMM uses Schmidt. 
            # Let's use the explicit 'k' calculation:
            
            for m in range(0, n + 1):
                if n == m:
                    P[n, m] = cd * P[n-1, m-1] * np.sqrt((2*n-1)/(2*n))
                    DP[n, m] = np.sqrt((2*n-1)/(2*n)) * (cd * DP[n-1, m-1] - sd * P[n-1, m-1])
                else:
                    nm = np.sqrt(n**2 - m**2)
                    root1 = np.sqrt(2*n - 1) / nm
                    root2 = np.sqrt(((n-1)**2 - m**2) / (2*n - 3))
                    
                    P[n, m] = (2*n - 1)/nm * sd * P[n-1, m] - root2 * (2*n - 1)/nm / (2*n - 3) * P[n-2, m] # This gets messy.
                    
                    # Fallback to the standard recurrence used in Geomag.c (WMM Reference)
                    # P[n,m] = (P[n-1,m] * sin(lat) * (2n-1) - P[n-2,m] * sqrt((n-1)^2 - m^2)) / sqrt(n^2 - m^2)
                    
                    val_1 = P[n-1, m] * sd * (2*n - 1)
                    val_2 = P[n-2, m] * np.sqrt((n-1)**2 - m**2)
                    P[n, m] = (val_1 - val_2) / np.sqrt(n**2 - m**2)
                    
                    d_val_1 = (DP[n-1, m] * sd + P[n-1, m] * cd) * (2*n - 1)
                    d_val_2 = DP[n-2, m] * np.sqrt((n-1)**2 - m**2)
                    DP[n, m] = (d_val_1 - d_val_2) / np.sqrt(n**2 - m**2)

        # 3. Summation
        X_p = 0.0
        Y_p = 0.0
        Z_p = 0.0
        
        ar_ratio = Re / r
        ar_n = ar_ratio * ar_ratio # (a/r)^2 start
        
        for n in range(1, self.max_deg + 1):
            ar_n *= ar_ratio # (a/r)^(n+2)
            
            sum_x = 0.0
            sum_y = 0.0
            sum_z = 0.0
            
            for m in range(0, n + 1):
                # Coeffs
                c = self.coeffs[(n,m)]
                g = c['g'] + dt * c['dg']
                h = c['h'] + dt * c['dh']
                
                cos_ml = np.cos(m * lon_geocentric)
                sin_ml = np.sin(m * lon_geocentric)
                
                # [cite_start]Main Terms [cite: 295, 299, 302]
                term = g * cos_ml + h * sin_ml
                d_term = g * sin_ml - h * cos_ml
                
                sum_x += term * DP[n, m]
                sum_y += m * d_term * P[n, m]
                sum_z += (n + 1) * term * P[n, m]
            
            X_p += ar_n * sum_x
            Y_p += ar_n * sum_y
            Z_p -= ar_n * sum_z # Z is negative gradient

        # [cite_start]4. Rotation from Geocentric to Geodetic (NED) [cite: 317]
        # X' = -Theta, Y' = Phi, Z' = -Radial
        # Correct directions
        Bt = -X_p
        Bp = Y_p 
        if cd > 1e-9: Bp = Bp / cd # Singularity check
        Br = Z_p
        
        # Rotate by difference in latitudes (psi)
        psi = rlat - lat_geocentric
        
        bx = Bt * np.cos(psi) - Br * np.sin(psi)
        by = Bp
        bz = Bt * np.sin(psi) + Br * np.cos(psi)
        
        # [cite_start]5. Compute Elements [cite: 325]
        H = np.sqrt(bx**2 + by**2)
        decl = np.degrees(np.arctan2(by, bx))
        incl = np.degrees(np.arctan2(bz, H))
        
        return decl, incl

    def generate_cpp_grid(self, min_lat, max_lat, min_lon, max_lon, steps, date_str):
        try:
            lat_steps=steps[0]
            lon_steps=steps[1]
        except:
            lat_steps=steps
            lon_steps=steps
        
        date_obj = datetime.datetime.strptime(date_str, "%Y-%m-%d").date()
        
        lats = np.linspace(min_lat, max_lat, lat_steps)
        lons = np.linspace(min_lon, max_lon, lon_steps)
        
        print("// WMM 2025 Table Generated for ESKF")
        print(f"// Date: {date_str} | Grid: {lat_steps}x{lon_steps}")
        print(f"// Lat: {min_lat} to {max_lat} | Lon: {min_lon} to {max_lon}")
        print("// Format: {inclination_rad, declination_rad }")
        print("namespace wmm {")
        print(f"constexpr float MAX_LAT = {max_lat};")
        print(f"constexpr float MIN_LAT = {min_lat};")
        print(f"constexpr float MAX_LON = {max_lon};")
        print(f"constexpr float MIN_LON = {min_lon};")
        print(f"constexpr int LAT_STEPS = {lat_steps};")
        print(f"constexpr int LON_STEPS = {lon_steps};")
        print(f"constexpr float  LAT_STEP_SIZE = (MAX_LAT - MIN_LAT) / LAT_STEPS;")
        print(f"constexpr float  LON_STEP_SIZE = (MAX_LON - MIN_LON) / LON_STEPS;")
        print(f"const float DATA[{lat_steps*lon_steps*2}] = {{")
        
        
        
        total = steps * steps
        lines=[]
        for lat in lats:
            data =[]
            for lon in lons:
                dec, inc = self.calculate(lat, lon, date_obj)
                data+=[np.deg2rad(inc),np.deg2rad(dec)]
            lines.append(", ".join(map(str, data)))
        print(",\n".join(lines))
        print("};")
        print("}")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Generate WMM Lookup Table")
    parser.add_argument("cof_file", help="Path to WMM.COF")
    parser.add_argument("output_file", help="Path to output header file")
    # You could add lat/lon bounds as arguments here too if you want them configurable
    args = parser.parse_args()

    # --- USER SETTINGS (Or make these arguments) ---
    DATE = "2025-12-15"
    MIN_LAT, MAX_LAT = 34.0, 36.0 
    MIN_LON, MAX_LON = -88.0, -85.0
    STEPS = 5 
    # ---------------------

    # Capture stdout to file
    original_stdout = sys.stdout
    try:
        with open(args.output_file, 'w') as f:
            sys.stdout = f
            wmm = WMM_Standalone(args.cof_file)
            wmm.generate_cpp_grid(MIN_LAT, MAX_LAT, MIN_LON, MAX_LON, STEPS, DATE)
    finally:
        sys.stdout = original_stdout