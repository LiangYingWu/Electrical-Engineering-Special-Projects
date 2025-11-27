import math
from pyproj import Proj, Transformer

# def latlngToXY(lat, lng, lat0, lng0):
#     deg_to_rad = math.pi / 180.0
#     dlat = (lat - lat0) * deg_to_rad
#     dlng = (lng - lng0) * deg_to_rad

#     r = 6378137.0
#     x = dlng * r * math.cos(lat0 * deg_to_rad)
#     y = dlat * r

#     return round(x, 10), round(y, 10)

def latlngToXY(lat, lng, lat0, lng0):
    # print("Converting lat/lng to x/y:", lat, lng)
    
    proj_wgs84 = Proj("epsg:4326")

    proj_local = Proj(
        proj='aeqd', 
        ellps='WGS84', 
        datum='WGS84', 
        lat_0=lat0, 
        lon_0=lng0
    )
    
    transformer = Transformer.from_proj(proj_wgs84, proj_local)
    x_coords, y_coords = transformer.transform(lat, lng)

    # print("x_coords:", x_coords, "y_coords:", y_coords)

    return round(x_coords, 10), round(y_coords, 10)

def processData(data, gps0, gps_error=(0,0)):
    # print("data", data)
    d = data.split(",")
    if len(d) < 16:
        return None
    
    data_processed = {}

    lat = float(d[1]) - gps_error[0]
    lng = float(d[2]) - gps_error[1]

    x, y = latlngToXY(lat, lng, gps0[0], gps0[1])
    data_processed[d[0]] = {
        "lat": lat, 
        "lng": lng, 
        "x": x, 
        "y": y
    }
    data_processed[d[3]] = {
        "degree": float(d[4]), 
    }
    data_processed[d[5]] = {
        "rps_avg": float(d[6]),
        "rpsA": float(d[7]), 
        "rpsB": float(d[8]), 
        "rpsC": float(d[9]) 
    }
    data_processed[d[10]] = {
        "1": float(d[11]), 
        "2": float(d[12]), 
        "3": float(d[13]), 
        "4": float(d[14]), 
        "5": float(d[15])
    }
    
    return data_processed