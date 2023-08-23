import {tiny} from './models/tiny-graphics.js';

const {
    Vector, Vector3, vec, vec3, vec4, color, Matrix, Mat4,
    Light, Shape, Material, Shader, Texture, Scene
} = tiny;

class Body {                      // **Body** can store and update the properties of a 3D body that incrementally
                                               
    constructor(shapes, relative_locations, materials, size, hitbox, children = []) {
        Object.assign(this,
            {shapes, relative_locations, materials, size, children, hitbox})
    }

    emplace(location_matrix, linear_velocity, angular_velocity, attached = true, ignore_collisions = false, spin_axis = vec3(0, 1, 0).normalized(), linear_acceleration = vec3(0, 0, 0), angular_acceleration = 0, activated = false) {                               // emplace(): assign the body's initial values, or overwrite them.
        this.center = location_matrix.times(vec4(0, 0, 0, 1)).to3();
        this.rotation = Mat4.translation(...this.center.times(-1)).times(location_matrix);
        this.previous = {center: this.center.copy(), rotation: this.rotation.copy()};
        // drawn_location gets replaced with an interpolated quantity:
        this.drawn_location = location_matrix;
        this.temp_matrix = Mat4.identity();
        this.enabled = true;
        return Object.assign(this, {
            linear_velocity,
            linear_acceleration,
            angular_velocity,
            angular_acceleration,
            spin_axis,
            attached,
            activated,
            ignore_collisions
        })
    }

    check_collision(b_hitbox, b_drawn_location) 
    {
        let a_AABB = this.calculate_AABB(this.hitbox, this.drawn_location);
        let b_AABB = this.calculate_AABB(b_hitbox, b_drawn_location);

        if (a_AABB === b_AABB)
            return 0;
        else if ((a_AABB[0][0] <= b_AABB[1][0] && a_AABB[1][0] >= b_AABB[0][0]) &&
            (a_AABB[0][1] <= b_AABB[1][1] && a_AABB[1][1] >= b_AABB[0][1]) &&
            (a_AABB[0][2] <= b_AABB[1][2] && a_AABB[1][2] >= b_AABB[0][2])) {
            return 1;
        }

        return 0;
    }


    calculate_AABB(hitbox = this.hitbox, drawn_location = this.drawn_location) {
        let transformed_hitbox = hitbox.map(x => drawn_location.times(x));
        let AABB = [vec4(...transformed_hitbox[0]), vec4(...transformed_hitbox[0])];

        for (let i = 1; i < transformed_hitbox.length; i++) {
            for (let j = 0; j < 3; j++) {
                if (transformed_hitbox[i][j] < AABB[0][j])
                    AABB[0][j] = transformed_hitbox[i][j];
                else if (transformed_hitbox[i][j] > AABB[1][j]) {
                    AABB[1][j] = transformed_hitbox[i][j];
                }
            }
        }
        return AABB;
    }

    advance(time_amount) {                 
        this.previous = {center: this.center.copy(), rotation: this.rotation.copy()};
        // Apply the velocities scaled proportionally to real time (time_amount):
        // Linear velocity first, then angular:
        this.center = this.center.plus(this.linear_velocity.times(time_amount));
        this.rotation.pre_multiply(Mat4.rotation(time_amount * this.angular_velocity, ...this.spin_axis));
    }

    blend_rotation(alpha) {  // blend_rotation(): Just naively do a linear blend of the rotations, which looks
        // ok sometimes but otherwise produces shear matrices, a wrong result.

        return this.rotation.map((x, i) => vec4(...this.previous.rotation[i]).mix(x, alpha));
    }

    blend_state(alpha) { // blend_state(): Compute the final matrix we'll draw using the previous two physical
    
        this.drawn_location = Mat4.scale(...this.size)
            .times(Mat4.translation(...this.previous.center.mix(this.center, alpha)))
            .times(this.blend_rotation(alpha));
    }

    // functions for testing a single point, & formulas
    static intersect_cube(p, margin = 0) {
        return p.every(value => value >= -1 - margin && value <= 1 + margin)
    }

    static intersect_sphere(p, margin = 0) {
        return p.dot(p) < 1 + margin;
    }

    check_if_colliding(b, collider) {    // check_if_colliding(): Collision detection function.
        
        if (this === b)
            return false;                     
                                              
        const T = this.inverse.times(b.drawn_location, this.temp_matrix);

        const {intersect_test, points, leeway} = collider;
        // For each vertex in that b, shift to the coordinate frame of
       
        return points.arrays.position.some(p =>
            intersect_test(T.times(p.to4(1)).to3(), leeway));
    }
}

export default Body;
