using UnityEngine;
using UnityEngine.Rendering.PostProcessing;

namespace HutongGames.PlayMaker.Actions
{

	[ActionCategory("Post Processing Stack V2")]
	[Tooltip("Modify Vignette effect During Runtime.")]
	public class PostProcessingVignette : FsmStateAction
	{
		#region public variables
				
		[RequiredField]
		[ObjectType(typeof(PostProcessProfile))]
		[Tooltip("Post Processing Profile to modify")]
		public FsmObject ObjectProfile;

		[Tooltip("Activate or deactivate Screen Space Reflections Layer")]
		public FsmBool ActiveLayer;
		[Tooltip("Activate or deactivate Screen Space Reflections Effect")]
		public FsmBool ActiveEffect;

		[Tooltip("Activate vignette mode parameter")]
		public FsmBool ModeActive;
		[Tooltip("Vignette mode")]
		public ParameterOverride<VignetteMode> Mode;

		[Tooltip("Activate Color modification")]
		public FsmBool ActivateColor;
		[Tooltip("Set the Color of the Vignette")]
		public FsmColor VignetteColor;

		[Tooltip("Activate Color modification")]
		public FsmBool ActivateMask;
		[Tooltip("Set the Color of the Vignette")]
		public FsmTexture Mask;

		[Tooltip("Activate Color modification")]
		public FsmBool ActivateCenter;
		[Tooltip("Set the Color of the Vignette")]
		public FsmVector2 Center;

		[Tooltip("Activate Color modification")]
		public FsmBool ActivateIntensity;
		[HasFloatSlider(0f, 1f)]
		[Tooltip("Set the Color of the Vignette")]
		public FsmFloat Intensity;

		[Tooltip("Activate Color modification")]
		public FsmBool ActivateSmoothness;
		[HasFloatSlider(0.01f, 1f)]
		[Tooltip("Set the smoothness")]
		public FsmFloat Smoothness;

		[Tooltip("Activate Color modification")]
		public FsmBool ActivateRoundness;
		[HasFloatSlider(0f, 1f)]
		[Tooltip("Set the Color of the Vignette")]
		public FsmFloat Roundness;

		[Tooltip("Activate Color modification")]
		public FsmBool ActivateRounded;
		[Tooltip("Set the Color of the Vignette")]
		public FsmBool Rounded;

		[Tooltip("Repeat every frame.")]
		public bool everyFrame;
		#endregion

		#region Private variables
		private Vignette VignetteLayer;
		private PostProcessProfile UsedProfile;		
		#endregion

		public override void Reset()
		{
			ActiveLayer= true;
			ActiveEffect = true;
			ObjectProfile = null;
			ModeActive = true;
			//Mode = 0;
			ActivateColor = true;
			VignetteColor = Color.black;
			ActivateCenter = true;
			Center = new Vector2(0.5f,0.5f);
			ActivateIntensity = true;
			Intensity = 0f;
			ActivateSmoothness = true;
			Smoothness = 0.2f;
			ActivateRoundness = true;
			Roundness = 1f;
			ActivateRounded = true;
			Rounded = false;
			everyFrame = false;
			
		}

		// Code that runs on entering the state.
		public override void OnEnter()
		{
			setVignette();
			if(!everyFrame)
			{
				Finish();
			}
		}

		// Code that runs every frame.
		public override void OnUpdate()
		{
			setVignette();
		}

		void setVignette()
		{
									
				UsedProfile = ObjectProfile.Value as PostProcessProfile;
				UsedProfile.TryGetSettings(out VignetteLayer);
				if(ObjectProfile.Value != null & UsedProfile.HasSettings<Vignette>())
				{
					if(VignetteLayer!=null)
					{
						VignetteLayer.active = ActiveLayer.Value; //Active AO Layer
						VignetteLayer.enabled.value = ActiveEffect.Value; // Active AO Effect
						VignetteLayer.mode.overrideState = ModeActive.Value;//Activate mode
						if(ModeActive.Value)
						{ 	
							VignetteLayer.mode.value = Mode; // mode 
						}
						VignetteLayer.color.overrideState = ActivateColor.Value;//Activate color
						if(ActivateColor.Value)
						{ 	
							VignetteLayer.color.value = VignetteColor.Value; // color 
						}
						VignetteLayer.mask.overrideState = ActivateMask.Value;//Activate Mask
						if(ActivateMask.Value)
						{ 	
							VignetteLayer.mask.value = Mask.Value; // Mask 
						}
						VignetteLayer.center.overrideState = ActivateCenter.Value;//Activate center
						if(ActivateCenter.Value)
						{ 	
							VignetteLayer.center.value = Center.Value; // center 
						}
						VignetteLayer.intensity.overrideState = ActivateIntensity.Value;//Activate intensity
						if(ActivateIntensity.Value)
						{ 	
							VignetteLayer.intensity.value = Intensity.Value; // intensity 
						}
						VignetteLayer.smoothness.overrideState = ActivateSmoothness.Value;//Activate Smoothness
						if(ActivateSmoothness.Value)
						{ 	
							VignetteLayer.smoothness.value = Smoothness.Value; // Smoothness 
						}
						VignetteLayer.roundness.overrideState = ActivateRoundness.Value;//Activate Roundness
						if(ActivateRoundness.Value)
						{ 	
							VignetteLayer.roundness.value = Roundness.Value; // Roundness 
						}
						VignetteLayer.rounded.overrideState = ActivateRounded.Value;//Activate Rounded
						if(ActivateRounded.Value)
						{ 	
							VignetteLayer.rounded.value = Rounded.Value; // Rounded 
						}
					}
				}
			}
		}
}


