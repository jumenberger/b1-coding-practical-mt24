from uuv_mission.mission_class import Mission

def main():
    # Adjust path if your folder name is different
    mission = Mission.from_csv("data/mission.csv")
    print(mission)
    print("\nSample Data:")
    print("Reference:", mission.reference.head())
    print("Cave Height:", mission.cave_height.head())
    print("Cave Depth:", mission.cave_depth.head())

if __name__ == "__main__":
    main()