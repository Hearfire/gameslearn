//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}


// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    // if (depth > this->maxDepth) {
    //     return Vector3f(0.0,0.0,0.0);
    // }
    Intersection intersection = Scene::intersect(ray);
    
    if(!intersection.happened){
        return Vector3f(0.0,0.0,0.0);
    }

    //hit light
    if(intersection.m->hasEmission()){
        return intersection.m->getEmission();
    }

    //hit object  

    Vector3f p0=intersection.coords; 
    Vector3f n=intersection.normal.normalized();
    
    //direct light
    float pdf_light;
    Intersection light_intersection;
    sampleLight(light_intersection,pdf_light);
    
    Vector3f x=light_intersection.coords;
    Vector3f ws=(x-p0).normalized();
    Vector3f NN=light_intersection.normal.normalized();
    Vector3f emit=light_intersection.emit;

    float d=(x-p0).norm();
    Vector3f p=(dotProduct(ray.direction,n)<0)?
        p0+n*EPSILON:
        p0-n*EPSILON;
    //Vector3f p=p0;

    Vector3f L_dir(0,0,0);
    Ray isblock(p,ws);
    if(intersect(isblock).distance-d>-0.0001){
        Vector3f eval=intersection.m->eval(ray.direction,ws,n);
        L_dir=emit*eval*dotProduct(ws,n)*dotProduct(-ws,NN)/d/d/pdf_light;       

    }

    //indirect light
    Vector3f L_indir(0,0,0);
    float rand=get_random_float();
    if(rand<RussianRoulette){
        Vector3f wi=intersection.m->sample(ray.direction,n).normalized();
        Ray nextray(p,wi);
        Intersection inte=intersect(nextray);
        if(inte.happened&&!inte.m->hasEmission()){
            float pdf_obj=intersection.m->pdf(ray.direction,wi,n);  

            if(pdf_obj>EPSILON){    
                Vector3f objshade=castRay(nextray,depth+1);
                Vector3f eval=intersection.m->eval(ray.direction,wi,n);            
                L_indir=objshade*eval*dotProduct(wi,n)/pdf_obj/RussianRoulette;
            }
        }

        

    }
    return L_dir+L_indir;



}

