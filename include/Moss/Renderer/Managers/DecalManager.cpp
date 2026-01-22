












void DecalManager::draw() {
    for (auto& decal : m_decals) {
        shader->bind();
        shader->setMat4("decalWorldToLocal", glm::inverse(decal.transform));
        shader->setTexture("decalTexture", decal.texture);
        // Bind G-buffer textures here
        drawbox(m_decals.transform); // Render cube at decal's location
    }
    shader->unbind();
}


void DecalManager::draw(VkCommandBuffer cmd, VkPipelineLayout pipelineLayout, VkDescriptorSet descriptorSet) {
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, decalPipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSet, 0, nullptr);

    for (const auto& decal : m_decals) {
            DecalPushConstant push{};
            push.worldToLocal = glm::inverse(decal.transform);
            push.color = decal.color.toVec4();
            push.blendFactor = decal.blendFactor;

        vkCmdPushConstants(cmd, pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(DecalPushConstant), &push);

            // You should have a cube mesh bound (vertex/index buffers)
        vkCmdDrawIndexed(cmd, cubeIndexCount, 1, 0, 0, 0);
    }
}