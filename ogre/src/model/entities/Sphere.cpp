#include "Sphere.hpp"

Sphere::Sphere(View& view)
    : Entity(view), sphere(view.createEntity(create(view.createManualObject()))) {
    sphere->setCastShadows(true);
    sphere->setMaterialName("Grey");

    sceneNode->setPosition(7, 14, 10);
    Ogre::Real scale = 0.05;
    sceneNode->setScale(scale, scale, scale);
    sceneNode->attachObject(sphere);
}

Ogre::MeshPtr Sphere::create(Ogre::ManualObject* manual) {
    const float r = 64;
    const int nRings = 64;
    const int nSegments = 64;

    manual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);

    float fDeltaRingAngle = (Ogre::Math::PI / nRings);
    float fDeltaSegAngle = (2 * Ogre::Math::PI / nSegments);
    unsigned short wVerticeIndex = 0;

    // Generate the group of rings for the sphere
    for (int ring = 0; ring <= nRings; ring++) {
        float r0 = r * sinf(ring * fDeltaRingAngle);
        float y0 = r * cosf(ring * fDeltaRingAngle);

        // Generate the group of segments for the current ring
        for (int seg = 0; seg <= nSegments; seg++) {
            float x0 = r0 * sinf(seg * fDeltaSegAngle);
            float z0 = r0 * cosf(seg * fDeltaSegAngle);

            // Add one vertex to the strip which makes up the sphere
            manual->position(x0, y0, z0);
            manual->normal(Ogre::Vector3(x0, y0, z0).normalisedCopy());
            manual->textureCoord((float)seg / (float)nSegments, (float)ring / (float)nRings);

            if (ring != nRings) {
                // each vertex (except the last) has six indicies pointing to it
                manual->index(wVerticeIndex + nSegments + 1);
                manual->index(wVerticeIndex);
                manual->index(wVerticeIndex + nSegments);
                manual->index(wVerticeIndex + nSegments + 1);
                manual->index(wVerticeIndex + 1);
                manual->index(wVerticeIndex);
                wVerticeIndex++;
            }
        }; // end for seg
    }      // end for ring
    manual->end();
    Ogre::MeshPtr mesh = manual->convertToMesh(manual->getName());
    mesh->_setBounds(
        Ogre::AxisAlignedBox(Ogre::Vector3(-r, -r, -r), Ogre::Vector3(r, r, r)), false);

    mesh->_setBoundingSphereRadius(r);
    unsigned short src, dest;
    if (!mesh->suggestTangentVectorBuildParams(Ogre::VES_TANGENT, src, dest)) {
        mesh->buildTangentVectors(Ogre::VES_TANGENT, src, dest);
    }

    return mesh;
}