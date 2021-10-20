using UnityEngine;
using UnityEngine.Rendering.PostProcessing;

namespace HutongGames.PlayMaker.Actions
{

	[ActionCategory("Post Processing Stack V2")]
	[Tooltip("Modify Chromatic Aberration During Runtime.")]
	public class PostProcessingChromaticAberration : FsmStateAction
	{
		#region public variables
				
		[RequiredField]
		[ObjectType(typeof(PostProcessProfile))]
		[Tooltip("Post Processing Profile to modify")]
		public FsmObject ObjectProfile;
		
		[Tooltip("Activate or deactivate Bloom Layer")]
		public FsmBool ActiveLayer;
		[Tooltip("Activate or deactivate Bloom Effect")]
		public FsmBool ActiveEffect;

		[Tooltip("Activate or deactivate DirtTexture parameter")]
		public FsmBool LutTextureActive;
		[Tooltip("Set Dirt Texture")]
		public FsmTexture LutTexture;
		[Tooltip("Activate or deactivate Dirt Texture intensity")]
		public FsmBool IntensityActive;
		[HasFloatSlider(0f, 100f)]
		[Tooltip("Dirt Texture Intensity")]
		public FsmFloat Intensity;
		[Tooltip("Activate or deactivate Fast mode")]
		public FsmBool FastModeActive;
		[Tooltip("Set Fast Mode")]
		public FsmBool FastMode;


		[Tooltip("Repeat every frame.")]
		public bool everyFrame;
		#endregion
		
		#region Private variables
		private ChromaticAberration AberrationLayer;
		private PostProcessProfile UsedProfile; 		
		#endregion

		public override void Reset()
		{
			ActiveLayer= true;
			ActiveEffect = true;
			ObjectProfile = null;
			LutTextureActive = true;
			LutTexture = null;
			IntensityActive= true;
			Intensity=0f;
			FastModeActive= true;
			FastMode= true;
			everyFrame = false;
		}

		// Code that runs on entering the state.
		public override void OnEnter()
		{
			SetChromaticAberration();
			if(!everyFrame)
			{
				Finish();
			}
		}

		// Code that runs every frame.
		public override void OnUpdate()
		{
			SetChromaticAberration();
		}

		void SetChromaticAberration()
		{
							
				UsedProfile = ObjectProfile.Value as PostProcessProfile;
				UsedProfile.TryGetSettings(out AberrationLayer);
				if(ObjectProfile != null & UsedProfile.HasSettings<ChromaticAberration>())
				{
					if(AberrationLayer!=null)
					{
						AberrationLayer.active = ActiveLayer.Value; //Active Chromatic Aberration Layer
						AberrationLayer.enabled.value = ActiveEffect.Value; // Active Chromatic Aberration Effect
						AberrationLayer.spectralLut.overrideState = LutTextureActive.Value ;//Activate LUT Texture
						if(LutTextureActive.Value)
						{
							AberrationLayer.spectralLut.value = LutTexture.Value ; // set LUT TExture
						}
						AberrationLayer.intensity.overrideState = IntensityActive.Value ;//Activate intensity
						if(IntensityActive.Value)
						{
							AberrationLayer.intensity.value = Intensity.Value ; // set intensity
						}
						AberrationLayer.fastMode.overrideState = FastModeActive.Value ;//Activate Fast Mode
						if(FastModeActive.Value)
						{
							AberrationLayer.fastMode.value = FastMode.Value ; // set Fast Mode
						}
				}
			}

		}
	}

}


