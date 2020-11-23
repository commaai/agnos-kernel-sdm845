// SPDX-License-Identifier: GPL-2.0

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <drm/drm_bootsplash.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_client.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_modes.h>

/* Draw a box with changing colors */
static void
drm_bootsplash_draw(struct drm_client_buffer *buffer, unsigned int sequence)
{
	unsigned int x, y;
	u32 *pix;

	pix = buffer->vaddr;
	pix += ((buffer->fb->height / 2) - 50) * buffer->fb->width;
	pix += (buffer->fb->width / 2) - 50;

	for (y = 0; y < 100; y++) {
		for (x = 0; x < 100; x++)
			*pix++ = drm_bootsplash_color_table[sequence];
		pix += buffer->fb->width - 100;
	}
}

static void drm_bootsplash_worker(struct work_struct *work)
{
	// struct drm_bootsplash *splash = container_of(work, struct drm_bootsplash, worker);
	// struct drm_event *event;
	// unsigned int i = 0, sequence = 0, fb_id;
	// int ret;

	// while (!splash->stop) {
	// 	/* Are we still in charge? */
	// 	// fb_id = drm_client_display_current_fb(splash->display);
	// 	// if (fb_id != splash->buffer[i]->fb_ids[0])
	// 	// 	break;

	// 	/*
	// 	 * We can race with userspace here between checking and doing
	// 	 * the page flip, so double buffering isn't such a good idea.
	// 	 * Tearing probably isn't a problem on a presumably small splash
	// 	 * animation. I've kept it to test the page flip code.
	// 	 */

	// 	i = !i;
	// 	drm_bootsplash_draw(splash->buffer[i], sequence++);
	// 	if (sequence == 3)
	// 		sequence = 0;

	// 	// ret = drm_client_display_page_flip(splash->display,
	// 	// 				   splash->buffer[i]->fb_ids[0],
	// 	// 				   true);
	// 	// if (!ret) {
	// 	// 	event = drm_client_read_event(splash->client, true);
	// 	// 	if (!IS_ERR(event))
	// 	// 		kfree(event);
	// 	// }
	// 	msleep(500);
	// }

	// for (i = 0; i < 2; i++)
	// 	drm_client_framebuffer_delete(splash->buffer[i]);
	// //drm_client_display_free(splash->display);
}

static int drm_bootsplash_setup(struct drm_bootsplash *splash)
{
    int ret = 0;
    struct drm_mode_set *modeset;
    struct drm_client_buffer *buffer;
    
    int i=0;
    drm_client_for_each_modeset(modeset, splash->client) {
        if(modeset->crtc->name)
            printk("BOOTSPLASH: mode %d (%s): x %d y %d. enabled: %d\n", i, modeset->crtc->name, modeset->crtc->x, modeset->crtc->y, modeset->crtc->enabled);
        else
            printk("BOOTSPLASH: mode %d (NO MODE NAME): x %d y %d\n", i, modeset->crtc->x, modeset->crtc->y);
        i++;
    }
    printk("BOOTSPLASH: found %d modes\n", i);

    // TODO: make generic
    buffer = drm_client_framebuffer_create(splash->client, 1080, 2160, DRM_FORMAT_RGBA8888);
    if(IS_ERR(buffer)){
        printk("BOOTSPLASH: framebuffer create error: %d\n", PTR_ERR(buffer));
        return -1;
    }
    printk("BOOTSPLASH: got buffer: %p\n", buffer);

    // TODO: put in loop
    drm_bootsplash_draw(buffer, 0);

    // Set backlight

	//schedule_work(&splash->worker);

	return 0;

// err_free_buffer:
// 	for (i--; i >= 0; i--)
// 		drm_client_framebuffer_delete(buffer[i]);
// err_free_display:
// 	drm_client_display_free(display);

// 	return ret;
}

static int drm_bootsplash_client_hotplug(struct drm_client_dev *client)
{
	int ret = 0;
    // struct drm_bootsplash *splash = drm_bootsplash_from_client(client);

    // if(!splash) {
    //     printk("BOOTSPLASH: splash is null!");
    //     return 0;
    // }

	// if (!splash->display)
	//     ret = drm_bootsplash_setup(splash);

	return ret;
}


static int drm_bootsplash_client_remove(struct drm_client_dev *client)
{
    // struct drm_bootsplash *splash = drm_bootsplash_from_client(client);

    // if(!splash) {
    //     printk("BOOTSPLASH: splash is null!");
    //     return 0;
    // }

	// if (splash->display) {
	// 	splash->stop = true;
	// 	flush_work(&splash->worker);
	// }

	// kfree(splash);

	return 0;
}

static const struct drm_client_funcs drm_bootsplash_client_funcs = {
	.owner		    = THIS_MODULE,
	.unregister		= drm_bootsplash_client_remove,
	.hotplug	    = drm_bootsplash_client_hotplug,
};

int drm_bootsplash_init(struct drm_device *dev)
{
    int ret = 0;
    struct drm_bootsplash *splash;

    // TODO: free these structs again!

	splash = kzalloc(sizeof(*splash), GFP_KERNEL);
	if (!splash)
		return -ENOMEM;
    
    splash->client = kzalloc(sizeof(struct drm_client_dev), GFP_KERNEL);
	if (!splash->client)
		return -ENOMEM;

    ret = drm_client_init(dev, splash->client, "drm_bootsplash", &drm_bootsplash_client_funcs);
    if (ret) {
        printk("BOOTSPLASH: drm_client_init setup failed: ret %d\n", ret);
        return ret;
    }

	// INIT_WORK(&splash->worker, drm_bootsplash_worker);

    ret = drm_bootsplash_setup(splash);

    if (ret) {
        printk("BOOTSPLASH: drm_bootsplash_setup failed: ret %d\n", ret);
        return ret;
    }

    drm_client_register(splash->client);

    return ret;
}
EXPORT_SYMBOL(drm_bootsplash_init);
