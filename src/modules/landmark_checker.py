import json
import math
import argparse


def load_landmarks(json_path):
    """Load landmarks from a JSON file."""
    with open(json_path, "r") as f:
        return json.load(f)


def distance(p1, p2):
    """Euclidean distance between two 2D points."""
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def find_nearby_landmarks(user_point, landmarks, max_distance):
    """Return the first landmark within max_distance of user_point."""
    for landmark in landmarks:
        lx = landmark["x"]
        ly = landmark["y"]
        d = distance(user_point, (lx, ly))

        if d <= max_distance:
            return {
                "name": landmark["name"],
                "description": landmark["description"],
                "distance": d
            }

    return None


def main():
    parser = argparse.ArgumentParser(description="Check proximity to landmarks.")
    parser.add_argument("x", type=float, help="X coordinate of the point")
    parser.add_argument("y", type=float, help="Y coordinate of the point")
    parser.add_argument("--json", dest="json_path", default="landmarks.json",
                        help="Path to the landmarks JSON file")
    parser.add_argument("--dist", type=float, default=5.0,
                        help="Maximum distance to count as 'near'")

    args = parser.parse_args()

    user_point = (args.x, args.y)
    landmarks = load_landmarks(args.json_path)

    result = find_nearby_landmarks(user_point, landmarks, args.dist)

    if result:
        print "Nearby Landmark Found!"
        print "Name: %s" % result["name"]
        print "Description: %s" % result["description"]
        print "Distance: %.2f" % result["distance"]
    else:
        print "No nearby landmarks."


if __name__ == "__main__":
    main()
