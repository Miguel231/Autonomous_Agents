class RayCastSensor:
    HIT = 0
    DISTANCE = 1
    OBJECT_INFO = 2
    ANGLE = 3

    def __init__(self, ray_perception_config):
        """
        :param ray_perception_config:
            [rays_per_direction, max_ray_degrees, sphere_cast_radius, ray_length]

            rays_per_direction -> Number of rays to the left and right of center.
                                  For example, a value of 2 means one ray in the center, two rays on the left and two on the right.
                                  The total number of rays will be always odd (1, 3, 5, 7...)
            max_ray_degrees -> Cone size for rays. Using 90 degrees casts rays to the left and right.
                               Greater than 90 degrees will go backwards.
                               Is the number of degrees from the center ray to the most left or most right ray.
            sphere_cast_radius -> Radius of sphere to cast.
            ray_length -> Length of the rays to cast.
        """
        self.sensor_rays = None
        self.rays_per_direction = ray_perception_config[0]
        self.num_rays = (self.rays_per_direction * 2) + 1
        self.max_ray_degrees = ray_perception_config[1]
        self.sphere_cast_radius = ray_perception_config[2]
        self.ray_length = ray_perception_config[3]

        # Array [3 x num_rays] with the live information of the sensor rays
        # row HIT -> bool, hit ON/OFF
        # row DISTANCE -> int, distance to the target
        # row OBJECT_INFO -> Information about the object that the ray is hitting
        # row ANGLE -> int, degrees from the center. Positive, rays on the right. Negative, rays on the left
        self.sensor_rays = [[False for _ in range(self.num_rays)],
                            [-1 for _ in range(self.num_rays)],
                            [None for _ in range(self.num_rays)],
                            [0.0 for _ in range(self.num_rays)]]
        # Fill the angles of each ray
        angle_between_rays = self.max_ray_degrees / self.rays_per_direction
        # Left side rays (negative angles)
        for r in range(self.rays_per_direction):
            self.sensor_rays[RayCastSensor.ANGLE][r] = -((self.rays_per_direction - r) * angle_between_rays)
        # Center ray
        self.sensor_rays[RayCastSensor.ANGLE][self.rays_per_direction] = 0.0
        # Right side rays (positive angles)
        for r in range(self.rays_per_direction+1, (self.rays_per_direction * 2)+1):
            self.sensor_rays[RayCastSensor.ANGLE][r] = ((r - self.rays_per_direction) * angle_between_rays)

    def set_perception(self, perception):
        """
        :param perception: Has the form  [[<num_ray_cast>, <hit[1\0]>, <hit_object_info>] ... ]
                           where <hit_object_info> is a dictionary with the form
                                {'name': <name_of_the_hit_object>, 'tag': <tag_of_the_hit_objrct>, 'distance': <distance_to_the_hit_object>}
                            or
                                None
                            if the ray does not hit any object
        :return:
        """
        for p in perception:
            self.sensor_rays[RayCastSensor.HIT][p[0]] = p[1]
            if p[2] is None:
                self.sensor_rays[RayCastSensor.DISTANCE][p[0]] = -1
            else:
                self.sensor_rays[RayCastSensor.DISTANCE][p[0]] = p[2]["distance"]
            self.sensor_rays[RayCastSensor.OBJECT_INFO][p[0]] = p[2]
