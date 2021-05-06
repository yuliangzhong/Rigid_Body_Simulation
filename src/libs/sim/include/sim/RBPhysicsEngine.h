#pragma once

#include <sim/RB.h>
#include <sim/RBRenderer.h>
#include <sim/RBSpring.h>
#include <sim/RBUtils.h>

namespace crl {

// subfunctions
Quaternion updateRotationGivenAngularVelocity(const Quaternion &q,
                                              const V3D &angularVelocity,
                                              double dt) {
    double angularVelocityMagnitude = angularVelocity.norm();
    // avoid divide by close to zero...
    if (angularVelocityMagnitude > 1e-10) 
    {
        // TODO: Ex.1 Integration
        // implement quaternion update logic
        // q_p = rot(w, dt) * q
        double n = angularVelocityMagnitude;
        double a1 = cos(dt*n/2);
        V3D a = sin(dt*n/2)*angularVelocity/n;
        Quaternion q_p = Quaternion(a1,a[0],a[1],a[2])*q;
        
        // TODO: fix the following line. 
        return q_p;
    }
    return q;
}

/**
 * our simulation world governed by the rigid-body dynamics
 */
class RBPhysicsEngine {
public:
    // constructor
    RBPhysicsEngine() {}

    // desctructor
    ~RBPhysicsEngine() {
        // we are going to delete every rigid body when simulation world is
        // destroyed.
        for (uint i = 0; i < rbs.size(); i++) delete rbs[i];
        rbs.clear();
        for (uint i = 0; i < springs.size(); i++) delete springs[i];
        springs.clear();
    }

    /**
     * add rigid body to simulation world.
     * note that we assume this is a cube block with approx. 0.24 x 0.24 x 0.24. 
     */
    RB *addRigidBodyToEngine() {
        rbs.push_back(new RB());
        // we use default mass = 100 kg
        rbs.back()->rbProps.mass = 100;
        rbs.back()->rbProps.id = rbs.size() - 1;
        // add force and torque
        f_ext.push_back(V3D());
        tau_ext.push_back(V3D());
        // add contact points (at vertices)
        rbs.back()->rbProps.contactPoints.push_back({P3D(0.125, 0.125, 0.125)});
        rbs.back()->rbProps.contactPoints.push_back(
            {P3D(0.125, 0.125, -0.125)});
        rbs.back()->rbProps.contactPoints.push_back(
            {P3D(0.125, -0.125, 0.125)});
        rbs.back()->rbProps.contactPoints.push_back(
            {P3D(0.125, -0.125, -0.125)});
        rbs.back()->rbProps.contactPoints.push_back(
            {P3D(-0.125, 0.125, 0.125)});
        rbs.back()->rbProps.contactPoints.push_back(
            {P3D(-0.125, 0.125, -0.125)});
        rbs.back()->rbProps.contactPoints.push_back(
            {P3D(-0.125, -0.125, 0.125)});
        rbs.back()->rbProps.contactPoints.push_back(
            {P3D(-0.125, -0.125, -0.125)});

        return rbs.back();
    }

    /**
     * add spring to simulation world.
     */
    RBSpring *addSpringToEngine(RB *parent, RB *child, P3D pJPos, P3D cJPos) {
        springs.push_back(new RBSpring());
        springs.back()->parent = parent;
        springs.back()->child = child;
        // local position of attached point from parent/child frames
        springs.back()->pJPos = pJPos;
        springs.back()->cJPos = cJPos;
        // we use default spring constant = 2000;
        springs.back()->k = 10000;

        // default rest length is the distance between attaching points when
        // the spring is added.
        if (parent == nullptr) {
            // TODO: Ex.2-1
            // implement your logic for a spring which is attached to world
            // 
            // TODO: Fix the following line
            P3D cJPos_world = child->state.getWorldCoordinates(cJPos);
            springs.back()->l0 = V3D(pJPos - cJPos_world).norm();
        } else {
            // TODO: Ex.2-2
            // implement your logic for a spring where both ends are attached
            // to rigid bodies
            // 
            // TODO: Fix the following line
            P3D pJPos_world = parent->state.getWorldCoordinates(pJPos);
            P3D cJPos_world = child->state.getWorldCoordinates(cJPos);

            springs.back()->l0 = V3D(pJPos_world-cJPos_world).norm();
        }
        return springs.back();
    }

    /**
     * apply external force (no spring force, no gravity. Force comes from 
     * third sources) to rigid body.
     */
    void applyForceTo(RB *rb, const V3D &f, const P3D &p) {
        // add force only if rb is in rbs
        for (uint i = 0; i < rbs.size(); i++) {
            if (rbs[i] == rb) {
                f_ext[i] += f;
                V3D r = rb->state.getWorldCoordinates(V3D(p));
                tau_ext[i] += r.cross(f);
            }
        }
    }

    /**
     * simulation stepping logic. advance one simulation timestep with dt.
     * the unit of dt is second. 
     */
    void step(double dt) {
        // external force and torque
        for (uint i = 0; i < rbs.size(); i++) {
            // force and torque by gravity
            f_ext[i] += rbs[i]->rbProps.mass * V3D(0, RBGlobals::g, 0);
        }

        // force and torque by springs
        for (RBSpring *spring : springs) {
            // TODO: Ex.2 Spring force
            // compute spring force f_spring = -kx and torque tau_spring and
            // add them to f_ext and tau_ext
            //
            // Hint:
            // - spring->l0 is the rest length and spring->k is the spring contant
            // - you can retrieve index of rb in this->rbs list from rb->rbProps.id

            if (spring->parent == nullptr) {
                // TODO: Ex.2-1
                // implement your logic for a spring which is attached to world
                P3D head = spring->pJPos;
                P3D tail = spring->child->state.getWorldCoordinates(spring->cJPos);
                V3D cJPos_world = spring->child->state.getWorldCoordinates(V3D(spring->cJPos));
                double mag_child = (spring->k)*(V3D(head - tail).norm()-spring->l0);
                V3D e_child = V3D(head - tail).normalized();

                int childID = spring->child->rbProps.id;
                f_ext[childID] += mag_child*e_child;
                tau_ext[childID] += cJPos_world.cross(mag_child*e_child);

            } else {
                // TODO: Ex.2-2
                // implement your logic for a spring where both ends are attached
                // to rigid bodies.
                P3D head = spring->parent->state.getWorldCoordinates(spring->pJPos);
                P3D tail = spring->child->state.getWorldCoordinates(spring->cJPos);
                V3D pJPos_world = spring->parent->state.getWorldCoordinates(V3D(spring->pJPos));
                V3D cJPos_world = spring->child->state.getWorldCoordinates(V3D(spring->cJPos));

                double mag_child = (spring->k)*(V3D(head - tail).norm()-spring->l0);
                V3D e_child = V3D(head - tail).normalized();

                int childID = spring->child->rbProps.id;
                int parentID = spring->parent->rbProps.id;
                f_ext[parentID] += -mag_child*e_child;
                tau_ext[parentID] += pJPos_world.cross(-mag_child*e_child);
                f_ext[childID] += mag_child*e_child;
                tau_ext[childID] += cJPos_world.cross(mag_child*e_child);
            }
        }

        for (uint i = 0; i < rbs.size(); i++) {
            RB *rb = rbs[i];
            // retrieve saved force and tau
            V3D f = f_ext[i];
            V3D tau = tau_ext[i];
            
            /*
            // check if go through the ground
            for(const RBContactPoint p : rb->rbProps.contactPoints)
            {
                P3D tmp = rb->state.getWorldCoordinates(p.localCoordinates);
                if(tmp.y < 0) printf("go through the ground! \n");
            }
            */

            // TODO: Ex.1 Integration
            // implement forward (explicit) Euler integration scheme for computing velocity and pose.
            //
            // Hint:
            // - recall, you need to compute 3x3 moment of inertia matrix expressed in world frame
            //
            // implement your logic here.
            //-----------------------------------------------------------------
            V3D v_new = rb->state.velocity + dt*f/rb->rbProps.mass;
            Matrix3x3 C_IB = rb->state.orientation.toRotationMatrix();
            Matrix3x3 I_world = C_IB*(rb->rbProps.MOI_local)*C_IB.inverse();
            V3D w = rb->state.angularVelocity;
            V3D w_new = w + dt*I_world.inverse()*(tau - w.cross(V3D(I_world*w)));
            //-----------------------------------------------------------------
            // Task1:

            // rb->state.pos += getP3D(dt*rb->state.velocity);
            // rb->state.orientation = updateRotationGivenAngularVelocity(rb->state.orientation,rb->state.angularVelocity,dt);
            // rb->state.velocity = v_new;
            // rb->state.angularVelocity = w_new;

            //-----------------------------------------------------------------

            // TODO: Ex.3 Stable Simulation
            // why our simulation is blown up? let's make it more stable!
            
            // Solution: Explicit Euler ---> Symplectic Euler

            // comment out (do not erase!) your logic for Ex.1 and implement Ex.3 here.
            
            //-----------------------------------------------------------------
            // Task3:

            rb->state.pos += getP3D(dt*v_new);
            rb->state.orientation = updateRotationGivenAngularVelocity(rb->state.orientation,w_new,dt);
            rb->state.velocity = v_new;
            rb->state.angularVelocity = w_new;
            //-----------------------------------------------------------------

            // TODO: Ex.4 Impulse-based Collisions
            // we will simulate collisions between rigidbodies and the ground plane.
            // implement impulse-based collisions here. use coefficient of
            // restituation "epsilon" and coefficient of friction "mu".
            // note that we ignore collisions between rigidbodies.
            //
            // Hint:
            // - read the material "ImpulseBasedCollisions" on CMM21 website carefully.
            // - detect collision by checking if the y coordinate of predefined
            // contact points < 0 (under the ground) or not. each rb has 8 contact points.
            // we will assume that collisions only happen at the contact points.
            // - there could be multiple contact points under the ground at the sametime,
            // but we will assume there's only one contact between a single ground~rb pair
            // at once: choose the contact points which is the lowest in y coordinate.

            // Ex.4 implementation here
            // updated pos, vel, ori, w
            if (simulateCollisions) 
            {
                P3D ra = P3D(0,999,0);

                for(const RBContactPoint point : rb->rbProps.contactPoints)
                {
                    P3D tmp = rb->state.getWorldCoordinates(point.localCoordinates);
                    if(tmp.y < ra.y) ra = tmp; 
                }
                if(ra.y < 0.0)
                {
                    // "A" for VECTORS and "a" for double
                    // compute U
                    V3D N = V3D(0,-1,0);
                    V3D U = rb->state.velocity + rb->state.angularVelocity.cross(V3D(ra));
                    double u_n = U.dot(N);
                    V3D U_n = u_n*N;
                    V3D U_t = U-U_n;
                    // compute K
                    Matrix3x3 C_IB_new = rb->state.orientation.toRotationMatrix();
                    Matrix3x3 I_world_new = C_IB_new*(rb->rbProps.MOI_local)*C_IB_new.inverse();
                    Matrix3x3 ra_x;  ra_x << 0,-ra.z,ra.y,ra.z,0,-ra.x,-ra.y,ra.x,0;
                    Matrix3x3 K = Matrix3x3::Identity(3,3)/rb->rbProps.mass - ra_x*I_world_new.inverse()*ra_x;
                    // compute J
                    V3D J;
                    J = K.inverse()*(-U-(double)eps*U_n);
                    // check friction zone
                    double j_n = J.dot(N);
                    if((J-j_n*N).norm() > mu*j_n)
                    {
                        V3D T = U_t.normalized();
                        double j_n_new = -(eps+1)*u_n/(N.dot(K*(N-(double)mu*T)));
                        J = j_n_new*N - mu*j_n_new*T;
                    }
                    // update linear and angular velocity
                    rb->state.velocity += J/rb->rbProps.mass;
                    rb->state.angularVelocity += I_world_new.inverse()*(V3D(ra).cross(J));
                    
                }
            }
        }

        // clean up
        for (uint i = 0; i < f_ext.size(); i++) {
            f_ext[i] = V3D();
            tau_ext[i] = V3D();
        }
    }

    /**
     * draw every rigid body belongs to world.
     */
    inline void draw(const gui::Shader &rbShader) {
        // draw moi boxes
        for (uint i = 0; i < this->rbs.size(); i++) {
            if (!this->rbs[i]->rbProps.fixed)
                crl::RBRenderer::drawMOI(this->rbs[i], rbShader);
        }

        // draw springs
        for (uint i = 0; i < this->springs.size(); i++) {
            P3D start, end;
            if (this->springs[i]->parent == nullptr) {
                start = this->springs[i]->pJPos;
                end = this->springs[i]->child->state.getWorldCoordinates(
                    this->springs[i]->cJPos);
            } else {
                start = this->springs[i]->parent->state.getWorldCoordinates(
                    this->springs[i]->pJPos);
                end = this->springs[i]->child->state.getWorldCoordinates(
                    this->springs[i]->cJPos);
            }
            drawCylinder(start, end, 0.05, rbShader);
        }

        // then draw collsion spheres
        if (showCollisionSpheres)
            for (uint i = 0; i < this->rbs.size(); i++)
                crl::RBRenderer::drawCollisionSpheres(this->rbs[i], rbShader);

        // and now coordinate frames
        if (showCoordFrame) {
            for (uint i = 0; i < this->rbs.size(); i++)
                crl::RBRenderer::drawCoordFrame(this->rbs[i], rbShader);
        }
    }

    /**
     * returns NULL if no RBs are hit by the ray...
     */
    RB *getFirstRBHitByRay(const Ray &ray, P3D &intersectionPoint) {
        RB *selectedRB = nullptr;
        double t = DBL_MAX;
        P3D tmpIntersectionPoint = P3D(0, 0, 0);

        for (uint i = 0; i < rbs.size(); i++) {
            if (rbs[i]->getRayIntersectionPoint(ray, tmpIntersectionPoint)) {
                double tTmp = ray.getRayParameterFor(tmpIntersectionPoint);
                if (tTmp < t) {
                    selectedRB = rbs[i];
                    t = tTmp;
                    intersectionPoint = tmpIntersectionPoint;
                }
            }
        }
        return selectedRB;
    }

public:
    // this is a list of all rigid bodies and springs belong to the world.
    std::vector<RB *> rbs;
    std::vector<RBSpring *> springs;

    // coefficients
    float eps = 0.0;  // restitution
    float mu = 0.8;   // friction

    // drawing flags
    bool showCollisionSpheres = false;
    bool showCoordFrame = false;

    // options
    bool simulateCollisions = false;

private:
    // list of force and tau applied to rigid bodies
    std::vector<V3D> f_ext;
    std::vector<V3D> tau_ext;
};
}  // namespace crl