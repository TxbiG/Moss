#ifndef MOSS_SUBVIEWPORT_VK_H
#define MOSS_SUBVIEWPORT_VK_H

#include <Moss/Renderer/Abstract/SubViewport.h>

struct [[nodiscard]] MOSS_API SubViewportVK : public SubViewport {
    SubViewportVK(int width, int height, CameraType type) : m_width(width), m_height(height), currentCamera(type)  { init(); }

    ~SubViewportVK() {
        if (colorImageView)     vkDestroyImageView(device, colorImageView, nullptr);
        if (colorImage)         vkDestroyImage(device, colorImage, nullptr);
        if (colorImageMemory)   vkFreeMemory(device, colorImageMemory, nullptr);
        if (depthImageView)     vkDestroyImageView(device, depthImageView, nullptr);
        if (depthImage)         vkDestroyImage(device, depthImage, nullptr);
        if (depthImageMemory)   vkFreeMemory(device, depthImageMemory, nullptr);
        if (framebuffer)        vkDestroyFramebuffer(device, framebuffer, nullptr);
        if (renderPass)         vkDestroyRenderPass(device, renderPass, nullptr);
    }


    void update() const override {
        switch (currentCamera) {
            case CameraType::ViewPortCamera2D: return camera2D.update();
            case CameraType::ViewPortCamera3D: return camera3D.update();
            default: MOSS_ERROR("Must be a 2D or 3D camera.") break;
        } 
    }

    void begin(VkCommandBuffer cmdBuffer, VkClearColorValue clearColor) override {
        VkClearValue clearValues[2] = {};
        clearValues[0].color = clearColor;
        clearValues[1].depthStencil = { 1.0f, 0 };

        VkRenderPassBeginInfo renderPassInfo = {};
        renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        renderPassInfo.renderPass = renderPass;
        renderPassInfo.framebuffer = framebuffer;
        renderPassInfo.renderArea.offset = { 0, 0 };
        renderPassInfo.renderArea.extent = { static_cast<uint32_t>(m_width), static_cast<uint32_t>(m_height) };
        renderPassInfo.clearValueCount = 2;
        renderPassInfo.pClearValues = clearValues;

        vkCmdBeginRenderPass(cmdBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);
    }

    void updateDescriptorSet(VkDescriptorSet descriptorSet, VkSampler sampler) const {
        VkDescriptorImageInfo imageInfo = {};
        imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        imageInfo.imageView = getColorTexture();
        imageInfo.sampler = sampler;

        VkWriteDescriptorSet write = {};
        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        write.dstSet = descriptorSet;
        write.dstBinding = 0;
        write.dstArrayElement = 0;
        write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        write.descriptorCount = 1;
        write.pImageInfo = &imageInfo;

        vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
    }


    void end(VkCommandBuffer cmdBuffer) override { vkCmdEndRenderPass(cmdBuffer); }

    void drawToScreen(VkCommandBuffer cmd, VkPipeline pipeline, VkDescriptorSet descriptorSet) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);

        // Assumes the fullscreen pipeline uses no vertex buffer (full-screen triangle)
        vkCmdBindDescriptorSets(
            cmd,
            VK_PIPELINE_BIND_POINT_GRAPHICS,
            pipelineLayout, // <- needs to be passed in or stored
            0,              // firstSet
            1,
            &descriptorSet,
            0,
            nullptr
        );

        // Draw a fullscreen triangle (3 vertices, no vertex/index buffers)
        vkCmdDraw(cmd, 3, 1, 0, 0);
    }



    VkDescriptorImageInfo getDescriptorImageInfo() const {
        VkDescriptorImageInfo info = {};
        info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        info.imageView = colorImageView;
        info.sampler = yourSampler; // Needs to be created and shared
        return info;
    }
    
    void drawToScreen(VkCommandBuffer cmdBuffer, VkPipeline pipeline, VkPipelineLayout pipelineLayout, VkDescriptorSet descriptorSet) {
        // Bind the pipeline
        vkCmdBindPipeline(cmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);

        // Bind the descriptor set that contains the color texture
        vkCmdBindDescriptorSets(cmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout,
                                0, 1, &descriptorSet, 0, nullptr);

        // Assuming a fullscreen quad vertex buffer is already bound or using a vertex shader that generates vertices (e.g. vertex shader with no vertex buffer)
        // If vertex buffer needed, bind here:
        // vkCmdBindVertexBuffers(...);

        // Draw the fullscreen quad (2 triangles, 4 vertices)
        vkCmdDraw(cmdBuffer, 4, 1, 0, 0);
    }

    VkImageView getColorTexture() const { return colorImageView; }

    int getWidth() const override { return m_width; }
    int getHeight() const override { return m_height; }

    Mat44 getViewProjMatrix() const override {
        switch (currentCamera) {
            case CameraType::ViewPortCamera2D: return camera2D.getViewProjMatrix();
            case CameraType::ViewPortCamera3D: return camera3D.getViewProjMatrix();
            default: return Mat44::sIdentity();
        }
    }

private:
    void init() {
        createImage(m_width, m_height, VK_FORMAT_R8G8B8A8_UNORM, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, colorImage, colorImageMemory);

        colorImageView = createImageView(colorImage, VK_FORMAT_R8G8B8A8_UNORM, VK_IMAGE_ASPECT_COLOR_BIT);

        // 2. Create depth image + view
        createImage(m_width, m_height, VK_FORMAT_D32_SFLOAT, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, depthImage, depthImageMemory);

        depthImageView = createImageView(depthImage, VK_FORMAT_D32_SFLOAT, VK_IMAGE_ASPECT_DEPTH_BIT);

        // 3. Create render pass
        VkAttachmentDescription colorAttachment = {};
        colorAttachment.format = VK_FORMAT_R8G8B8A8_UNORM;
        colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
        colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        colorAttachment.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        VkAttachmentDescription depthAttachment = {};
        depthAttachment.format = VK_FORMAT_D32_SFLOAT;
        depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
        depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        depthAttachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        VkAttachmentReference colorRef = { 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };
        VkAttachmentReference depthRef = { 1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL };

        VkSubpassDescription subpass = {};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &colorRef;
        subpass.pDepthStencilAttachment = &depthRef;

        std::array<VkAttachmentDescription, 2> attachments = { colorAttachment, depthAttachment };

        VkRenderPassCreateInfo rpInfo = {};
        rpInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        rpInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
        rpInfo.pAttachments = attachments.data();
        rpInfo.subpassCount = 1;
        rpInfo.pSubpasses = &subpass;

        if (vkCreateRenderPass(device, &rpInfo, nullptr, &renderPass) != VK_SUCCESS) { MOSS_ERROR("Failed to create render pass for SubViewport!"); }

        // 4. Create framebuffer
        std::array<VkImageView, 2> views = { colorImageView, depthImageView };

        VkFramebufferCreateInfo fbInfo = {};
        fbInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        fbInfo.renderPass = renderPass;
        fbInfo.attachmentCount = static_cast<uint32_t>(views.size());
        fbInfo.pAttachments = views.data();
        fbInfo.width = m_width;
        fbInfo.height = m_height;
        fbInfo.layers = 1;

        if (vkCreateFramebuffer(device, &fbInfo, nullptr, &framebuffer) != VK_SUCCESS) {
            MOSS_ERROR("Failed to create framebuffer for SubViewport!");
        }
    }
    VkRenderPass renderPass;
    VkFramebuffer framebuffer;

    VkImage colorImage;
    VkDeviceMemory colorImageMemory;
    VkImageView colorImageView;

    VkImage depthImage;
    VkDeviceMemory depthImageMemory;
    VkImageView depthImageView;

    Camera2D camera2D;
    Camera3D camera3D;
    CameraType currentCamera;

    int m_width, m_height;
};

#endif // MOSS_SUBVIEWPORT_VK_H